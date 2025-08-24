// Combined: Carbase (Mecanum wheels) + Shooter + Clamp
// Single CAN (CAN0). Wheels idx 0..3, Shooter idx 4, Reloader idx 5.

#include <Arduino.h>
#include "MecanumWheelDriver.h"
#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>
#include <Wire.h>

// ---------------- CAN0 instance ----------------
ESP32CAN CAN0; // 如需特定建構參數，告訴我

// ---------------- pins & constants ----------------
#define MOTOR_NUM 4

#define lockPin   12   // 電磁鎖（shooter）
#define clampPin  13   // 夾爪輸出
#define sensorPin 32   // homing 感測器

#define LOCKED LOW
#define UNLOCKED HIGH

#define CLAMP_CLOSED_LEVEL HIGH
#define CLAMP_OPEN_LEVEL   LOW

#define POSITION_TOLERANCE 400L
#define JOYSTICK_DEADZONE 12

const byte thisAddress = 0x09; // I2C address for distance source
const float DIST_MIN = 90.0f;
const float DIST_MAX = 110.0f;
const unsigned long UNLOCK_AUTO_TIMEOUT = 2000UL;
const unsigned long ARM_TIMEOUT = 5000UL;
const unsigned long HOMED_DELAY = 2000UL;

// ---------------- objects ----------------
MecanumWheelDriver motor_driver(-1.0, -1.0, -1.0);
Rm_set motors;

// RM target rpm buffer for wheels (0..3)
int rm_target_speed[MOTOR_NUM];

// ---------------- I2C rx struct ----------------
struct I2cRxStruct { float Distance; byte padding[10]; };
volatile I2cRxStruct rxData;
volatile bool newRxData = false;

// ---------------- shooter variables (index 4) ----------------
volatile int homing_stage = -1;
volatile int initial_stage = -1;
volatile long initial_encoder = 0;
volatile bool shot_requested = false;
bool shot_in_progress = false;
long shot_target_pos = 0;
bool lockState = UNLOCKED;

enum MachineState { STATE_HOMING, STATE_HOMED_LOCKED, STATE_MOVING_TO_SHOT, STATE_AT_SHOT_LOCKED, STATE_UNLOCKED, STATE_RETURNING_VIA_SENSOR };
volatile MachineState state = STATE_HOMING;

int top = -400000; // offset to shot pos (adjust as needed)
unsigned long home_start_time = 0;

// armed flags
volatile bool armedBySquare = false;
volatile bool armedByUp = false;
unsigned long arm_timestamp = 0;

// ---------------- clamp / reloader (index 5) ----------------
volatile bool clampClosed = false; // true = closed (CLAMP_CLOSED_LEVEL)
long int_pos = 0;      // latched home position for clamp/reloader (will be set in setup after CAN delay)
long current_pos = 0;

int pos_A = 120000;
int pos_B = 90000;

// reloader state
long initial_pos_reload = 0, target_pos_reload = 0;
bool reload_in_progress = false, reload_done = false;
int reload_step = 0;
unsigned long step_timer = 0;
const long RELOAD_OFFSET = 20000;

// clamp auto state (for pos/ shake)
enum AutoState { IDLE, MOVING_UP, CLAMP_SHAKE, MOVING_DOWN, DONE };
AutoState autoState = IDLE;
unsigned long stateTimer = 0;
int shakeCounter = 0;
int shakeDelay = 100;

// ---------------- PS4 ----------------
int lx = 0, ly = 0, rx = 0, ry = 0;
bool current_buttons[16] = {0}, last_buttons[16] = {0};
int press_list[16] = {0};

// ---------------- timing ----------------
unsigned long last_dbg_time = 0;

// ---------------- forward decl ----------------
void send_rm_frame();
void task_cantx(void* param);
void can0_callback(CAN_FRAME *frame);
void notify();
void sendJoystickFeedback();
void update_wheel_targets_from_joystick();

// ---------------- ISR ----------------
void IRAM_ATTR sensorISR() {
  if (homing_stage > 0) {
    homing_stage = 0;
    initial_stage = 0;
  }
}

// ---------------- I2C receive ----------------
void receiveEvent(int numBytesReceived) {
  if (!newRxData) {
    Wire.readBytes((byte*)&rxData, numBytesReceived);
    newRxData = true;
  } else {
    while (Wire.available() > 0) Wire.read();
  }
}

// ---------------- send RM frames (uses motors.can_msg0/1) ----------------
void send_rm_frame() {
  CAN_FRAME tx0, tx1;
  tx0.id = 0x200; tx0.length = 8;
  tx1.id = 0x1FF; tx1.length = 8;
  for (int i = 0; i < 8; ++i) {
    tx0.data.byte[i] = motors.can_msg0[i];
    tx1.data.byte[i] = motors.can_msg1[i];
  }
  CAN0.sendFrame(tx0);
  CAN0.sendFrame(tx1);
}

// ---------------- CAN TX task ----------------
void task_cantx(void* param) {
  for (;;) {
    send_rm_frame();
    vTaskDelay(5 / portTICK_PERIOD_MS); // 5 ms
  }
}

// ---------------- mecanum -> set wheel targets ----------------
void update_wheel_targets_from_joystick() {
  motor_driver.getMovement(rm_target_speed, -lx/127.0, ly/127.0, -rx/127.0);
  for (int i = 0; i < MOTOR_NUM; ++i) {
    motors.set_target_rpm(i, rm_target_speed[i]); // indices 0..3 are wheels
  }
}

// ---------------- joystick feedback ----------------
void sendJoystickFeedback() {
  CAN_FRAME joy_feedback;
  joy_feedback.id = 0xF9;
  joy_feedback.length = 8;
  joy_feedback.data.byte[0] = min(PS4.LStickX() + 128, 255);
  joy_feedback.data.byte[1] = min(PS4.LStickY() + 128, 255);
  joy_feedback.data.byte[2] = min(PS4.RStickX() + 128, 255);
  joy_feedback.data.byte[3] = min(PS4.RStickY() + 128, 255);
  for (int i=4;i<8;i++) joy_feedback.data.byte[i] = 0;
  CAN0.sendFrame(joy_feedback);
}

// ---------------- CAN0 callback (handle feedback for all motors) ----------------
void can0_callback(CAN_FRAME *frame) {
  int rx_id = frame->id;
  // only process if id in 0x201..0x208 (RM feedback)
  if (rx_id >= 0x201 && rx_id <= 0x208) {
    int motor_index = rx_id - 0x201; // maps to motors.gearbox_pos[]
    motors.update_motor_status(motor_index, micros(),
      frame->data.uint8[0] << 8 | frame->data.uint8[1],
      frame->data.uint8[2] << 8 | frame->data.uint8[3]);
  }

  // Homing: while homing_stage==1, drive shooter toward sensor
  if (homing_stage == 1) {
    motors.set_target_rpm(4, 1000); // shooter at index 4
  }

  // Latch home when ISR signalled initial_stage==0
  if (initial_stage == 0) {
    motors.set_target_rpm(4, 0);
    initial_encoder = motors.gearbox_pos[4];
    initial_stage = 1;
    shot_requested = true; // optional auto shot if desired
    home_start_time = millis();
    digitalWrite(lockPin, LOCKED);
    lockState = LOCKED;
    state = STATE_HOMED_LOCKED;
    Serial.printf("HOMED: initial_encoder=%ld\n", initial_encoder);
  }
}

// ---------------- shooter logic ----------------
void shooter_logic(float distance_local) {
  // Auto move to shot after homed
  if (shot_requested && !shot_in_progress && (millis() - home_start_time >= HOMED_DELAY)) {
    shot_target_pos = initial_encoder + top;
    motors.set_target_pos(4, shot_target_pos);
    shot_in_progress = true;
    shot_requested = false;
    state = STATE_MOVING_TO_SHOT;
    Serial.printf("AUTO: moving to shot pos=%ld\n", shot_target_pos);
  }

  // Arrival detection
  if (state == STATE_MOVING_TO_SHOT) {
    long cur = motors.gearbox_pos[4];
    long diff = (cur > shot_target_pos) ? (cur - shot_target_pos) : (shot_target_pos - cur);
    if (diff <= POSITION_TOLERANCE) {
      shot_in_progress = false;
      state = STATE_AT_SHOT_LOCKED;
      Serial.println("Arrived at shot position -> AT_SHOT_LOCKED");
    }
  }

  // Armed-by-square or by-up: wait for distance in range
  if ((armedBySquare || armedByUp) && state == STATE_AT_SHOT_LOCKED) {
    if (millis() - arm_timestamp > ARM_TIMEOUT) {
      armedBySquare = false; armedByUp = false;
      Serial.println("Arm timed out -> cleared");
    } else {
      if (distance_local > DIST_MIN && distance_local < DIST_MAX) {
        if (clampClosed) {
          digitalWrite(lockPin, UNLOCKED);
          lockState = UNLOCKED;
          state = STATE_UNLOCKED;
          armedBySquare = false; armedByUp = false;
          Serial.println("Auto UNLOCK triggered by distance (armed).");
          motors.set_target_rpm(4, 0);
        } else {
          Serial.println("Distance in range but clamp not closed -> not unlocking");
        }
      }
    }
  }

  // Auto-return when unlocked for too long
  static unsigned long unlock_time_marker = 0;
  if (state == STATE_UNLOCKED) {
    if (unlock_time_marker == 0) unlock_time_marker = millis();
    if (millis() - unlock_time_marker >= UNLOCK_AUTO_TIMEOUT) {
      Serial.println("UNLOCK timeout -> start return via sensor");
      digitalWrite(lockPin, UNLOCKED); lockState = UNLOCKED;
      homing_stage = 1;
      state = STATE_RETURNING_VIA_SENSOR;
      unlock_time_marker = 0;
    }
  } else {
    unlock_time_marker = 0;
  }
}

// ---------------- reloader logic (motor idx 5) ----------------
void startReload() {
  reload_in_progress = true; reload_step = 0; step_timer = millis(); reload_done = false;
  Serial.println("Reload started");
}
void reloader_logic() {
  if (!reload_in_progress) return;
  unsigned long now = millis();
  switch(reload_step) {
    case 0:
      motors.set_target_pos(5, target_pos_reload);
      if (abs(motors.gearbox_pos[5]-target_pos_reload) < 500) { reload_step++; step_timer=now; }
      break;
    case 1:
      digitalWrite(clampPin, CLAMP_OPEN_LEVEL); clampClosed = false;
      if (now - step_timer > 300) { reload_step++; step_timer = now; }
      break;
    case 2:
      digitalWrite(clampPin, CLAMP_CLOSED_LEVEL); clampClosed = true;
      if (now - step_timer > 300) { reload_step++; step_timer = now; }
      break;
    case 3:
      digitalWrite(clampPin, CLAMP_OPEN_LEVEL); clampClosed = false;
      if (now - step_timer > 300) { reload_step++; step_timer = now; }
      break;
    case 4:
      digitalWrite(clampPin, CLAMP_CLOSED_LEVEL); clampClosed = true;
      if (now - step_timer > 300) { reload_step++; step_timer = now; }
      break;
    case 5:
      motors.set_target_pos(5, initial_pos_reload);
      if (abs(motors.gearbox_pos[5] - initial_pos_reload) < 500) { reload_step++; }
      break;
    case 6:
      reload_in_progress = false; reload_done = true;
      Serial.println("Reload done");
      break;
  }
}

// ---------------- clamp auto state (pos/ shake) ----------------
void clamp_logic() {
  current_pos = motors.gearbox_pos[5];

  switch (autoState) {
    case IDLE: break;
    case MOVING_UP:
      motors.set_target_pos(5, int_pos + pos_A);
      if (abs(current_pos - (int_pos + pos_A)) < 500) {
        Serial.println("Clamp reached pos_A -> opening clamp then shake");
        clampClosed = false; digitalWrite(clampPin, CLAMP_OPEN_LEVEL);
        stateTimer = millis(); shakeCounter = 0; autoState = CLAMP_SHAKE;
      }
      break;
    case CLAMP_SHAKE:
      if (millis() - stateTimer > shakeDelay) {
        clampClosed = !clampClosed;
        digitalWrite(clampPin, clampClosed ? CLAMP_CLOSED_LEVEL : CLAMP_OPEN_LEVEL);
        shakeCounter++; stateTimer = millis();
        if (shakeCounter >= 4) {
          Serial.println("Clamp shake done -> moving down to pos_B");
          motors.set_target_pos(5, int_pos + pos_B);
          autoState = MOVING_DOWN;
        }
      }
      break;
    case MOVING_DOWN:
      if (abs(current_pos - (int_pos + pos_B)) < 500) {
        Serial.println("Clamp auto sequence done -> IDLE");
        autoState = IDLE; shakeCounter = 0;
      }
      break;
    default: break;
  }
}

// ---------------- PS4 notify (edge detection) ----------------
void notify() {
  // we don't read wheel movement here (we read in loop for continuous control)
  current_buttons[0] = PS4.Right();
  current_buttons[1] = PS4.Up();
  current_buttons[2] = PS4.Left();
  current_buttons[3] = PS4.Down();
  current_buttons[4] = PS4.Circle();  // CIRCLE -> clamp toggle (as requested)
  current_buttons[5] = PS4.Triangle();
  current_buttons[6] = PS4.Square();
  current_buttons[7] = PS4.Cross();
  current_buttons[8] = PS4.L1();
  current_buttons[9] = PS4.R1();
  current_buttons[10] = PS4.L2();
  current_buttons[11] = PS4.R2();
  current_buttons[12] = PS4.Options();
  current_buttons[13] = PS4.Share();
  current_buttons[14] = PS4.PSButton();
  current_buttons[15] = PS4.Touchpad();

  for (int i=0;i<16;i++) {
    press_list[i] = (int)current_buttons[i] - (int)last_buttons[i];
    last_buttons[i] = current_buttons[i];
  }

  // Down (index 3) -> move reloader/clamp to int_pos
  if (press_list[3] == 1) {
    Serial.println("DOWN pressed -> move to int_pos");
    motors.set_target_pos(5, int_pos);
    autoState = IDLE; // cancel any auto clamp sequence
  }

  // Circle (index 4) -> manual clamp toggle (if safe)
  if (press_list[4] == 1) {
    if (state == STATE_UNLOCKED || state == STATE_MOVING_TO_SHOT || state == STATE_RETURNING_VIA_SENSOR || shot_in_progress) {
      Serial.println("Cannot toggle clamp during unlocked / moving / shooting");
    } else {
      clampClosed = !clampClosed;
      digitalWrite(clampPin, clampClosed ? CLAMP_CLOSED_LEVEL : CLAMP_OPEN_LEVEL);
      Serial.print("Circle: clamp toggled -> closed? "); Serial.println(clampClosed ? "YES":"NO");
    }
  }

  // Up -> arm when at shot locked
  if (press_list[1] == 1 && state == STATE_AT_SHOT_LOCKED) {
    if (clampClosed) {
      armedByUp = true;
      arm_timestamp = millis();
      Serial.println("Armed by UP");
      PS4.setRumble(100,100); PS4.setLed(0,255,0); PS4.sendToController();
      delay(80);
      PS4.setRumble(0,0); PS4.setLed(255,0,0); PS4.sendToController();
    } else {
      Serial.println("Cannot arm: clamp not closed");
    }
  }

  // Square -> armBySquare (distance)
  if (press_list[6] == 1 && state == STATE_AT_SHOT_LOCKED) {
    if (clampClosed) {
      armedBySquare = true;
      arm_timestamp = millis();
      Serial.println("Armed by SQUARE (waiting for distance)");
    } else {
      Serial.println("SQUARE pressed but clamp not closed -> ignored");
    }
  }

  // Triangle -> direct shoot (only if at shot locked, with safety checks)
  if (press_list[5] == 1) {
    if (state == STATE_AT_SHOT_LOCKED) {
      if (!clampClosed) { Serial.println("Abort fire: clamp not closed"); }
      else if (lockState != LOCKED) { Serial.println("Abort fire: lock not locked"); }
      else {
        shot_requested = true;
        Serial.println("Triangle -> shot requested");
      }
    } else {
      Serial.println("Triangle pressed but shooter not at shot locked");
    }
  }

  // Square (index 6) reload handled above; Cross (index 7) unused or for future
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(sensorPin, INPUT);
  pinMode(lockPin, OUTPUT);
  pinMode(clampPin, OUTPUT);

  digitalWrite(lockPin, UNLOCKED);
  lockState = UNLOCKED;
  digitalWrite(clampPin, CLAMP_OPEN_LEVEL);
  clampClosed = false;

  Wire.begin(thisAddress);
  Wire.onReceive(receiveEvent);

  PS4.begin("a0:a0:a0:a0:a0:a0");
  PS4.attach(notify);
  PS4.attachOnConnect([](){ Serial.println("PS4 connected"); });
  PS4.attachOnDisconnect([](){ Serial.println("PS4 disconnected"); });

  // init motors (reset gearbox positions and set PID)
  for (int i = 0; i < motors.MOTOR_NUM; ++i) {
    motors.set_pid_rpm(i, 6, 0.01, 0);
    motors.reset_gearbox_pos(i);
  }

  // CAN0 (single CAN)
  CAN0.begin(1000000);
  CAN0.watchFor(); // accept all IDs (we filter in callback)
  CAN0.setGeneralCallback(can0_callback);
  Serial.println("CAN0 started");

  // give CAN callback a short time to populate gearbox_pos values, then latch int_pos
  delay(50);
  int_pos = motors.gearbox_pos[5];
  initial_pos_reload = int_pos;
  target_pos_reload = initial_pos_reload + RELOAD_OFFSET;
  Serial.printf("Recorded int_pos (clamp/home) = %ld\n", int_pos);

  // attach sensor interrupt properly
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorISR, FALLING);

  // initial homing for shooter
  homing_stage = 1;
  state = STATE_HOMING;

  // start RM TX task
  xTaskCreatePinnedToCore(task_cantx, "cantx", 4096, NULL, 1, NULL, 1);

  Serial.println("Setup complete.");
}

// ---------------- main loop ----------------
void loop() {
  // continuous wheel control if PS4 connected
  if (PS4.isConnected()) {
    lx = PS4.LStickX();
    ly = PS4.LStickY();
    rx = PS4.RStickX();
    update_wheel_targets_from_joystick();
    sendJoystickFeedback();
  }

  // read I2C distance if available
  static float distance_local = 0.0f;
  if (newRxData) {
    noInterrupts();
    distance_local = rxData.Distance;
    newRxData = false;
    interrupts();
  }

  // call higher-level logic
  reloader_logic();
  clamp_logic();
  shooter_logic(distance_local);

  // periodic debug
  if (millis() - last_dbg_time > 300) {
    last_dbg_time = millis();
    Serial.printf("STATE=%d shooterPos=%ld clampPos=%ld clampClosed=%d armedUp=%d armedSq=%d lock=%d\n",
      (int)state, motors.gearbox_pos[4], motors.gearbox_pos[5], (int)clampClosed, (int)armedByUp, (int)armedBySquare, digitalRead(lockPin));
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);
}
