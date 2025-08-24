// Shooter + Clamp only (ESP32) - single CAN (CAN0) version
#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>
#include <Wire.h>

// ---------- pins & constants ----------
#define lockPin   13   // 電磁鎖
#define clampPin  14   // 夾爪輸出
#define sensorPin 32   // homing 感測器

#define LOCKED LOW
#define UNLOCKED HIGH

#define CLAMP_CLOSED_LEVEL HIGH
#define CLAMP_OPEN_LEVEL   LOW

#define POSITION_TOLERANCE 400L
#define JOYSTICK_DEADZONE 12

// I2C addr for distance receiver
const byte thisAddress = 0x09;
const float DIST_MIN = 90.0f;
const float DIST_MAX = 110.0f;
const unsigned long UNLOCK_AUTO_TIMEOUT = 2000UL;
const unsigned long ARM_TIMEOUT = 5000UL; // armedByUp 超時
const unsigned long homed_Delay = 2000UL;

// ---------- objects ----------
Rm_set motors;

// ---------- I2C rx struct ----------
struct I2cRxStruct {
  float Distance;
  byte padding[10];
};
volatile I2cRxStruct rxData;
volatile bool newRxData = false;

// ---------- shooter variables (motor idx 4) ----------
volatile int homing_stage = -1;
volatile int initial_stage = -1;
volatile long initial_encoder = 0;
volatile bool shot_requested = false;
bool shot_in_progress = false;
long shot_target_pos = 0;
bool lockState = UNLOCKED;

enum MachineState {
  STATE_HOMING,
  STATE_HOMED_LOCKED,
  STATE_MOVING_TO_SHOT,
  STATE_AT_SHOT_LOCKED,
  STATE_UNLOCKED,
  STATE_RETURNING_VIA_SENSOR
};
volatile MachineState state = STATE_HOMING;

int top = -400000; // offset to shot pos (保持原先 sign)
unsigned long home_start_time = 0;

// armed flags
volatile bool armedBySquare = false;
volatile bool armedByUp = false;
unsigned long arm_timestamp = 0;

// ---------- clamp variables (motor idx 5) ----------
volatile bool clampClosed = false; // true = closed (CLAMP_CLOSED_LEVEL)
long int_pos = 0;
long current_pos = 0;

int pos_A = 120000;
int pos_B = 90000;

// clamp state machine
enum AutoState { IDLE, MOVING_UP, WAIT_RELEASE, CLAMP_SHAKE, MOVING_DOWN, DONE };
AutoState autoState = IDLE;
unsigned long stateTimer = 0;
int shakeCounter = 0;
int shakeDelay = 100;

// ---------- PS4 ----------
int lx, ly, rx, ry;
bool current_buttons[16];
bool last_buttons[16];
int press_list[16];

// ---------- CAN / task timing ----------
unsigned long last_time = 0;

// ---------- forward decl ----------
void send_rm_frame();
void task_cantx(void* param);

// ---------- ISR ----------
void IRAM_ATTR sensorISR() {
  if (homing_stage > 0) {
    homing_stage = 0;
    initial_stage = 0; // will be processed inside CAN callback (latch encoder)
  }
}

// ---------- I2C receive ----------
void receiveEvent(int numBytesReceived) {
  if (!newRxData) {
    Wire.readBytes((byte*)&rxData, numBytesReceived);
    newRxData = true;
  } else {
    while (Wire.available() > 0) Wire.read();
  }
}

// ---------- PS4 notify ----------
void notify() {
  lx = PS4.LStickX();
  ly = PS4.LStickY();
  rx = PS4.RStickX();
  ry = PS4.RStickY();

  if (abs(lx) < JOYSTICK_DEADZONE) lx = 0;
  if (abs(ly) < JOYSTICK_DEADZONE) ly = 0;
  if (abs(rx) < JOYSTICK_DEADZONE) rx = 0;
  if (abs(ry) < JOYSTICK_DEADZONE) ry = 0;

  current_buttons[0] = PS4.Right();
  current_buttons[1] = PS4.Up();
  current_buttons[2] = PS4.Left();
  current_buttons[3] = PS4.Down();

  current_buttons[4] = PS4.Circle();
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

  for (int i = 0; i < 16; i++) {
    press_list[i] = (int)current_buttons[i] - (int)last_buttons[i];
  }
  for (int i = 0; i < 16; i++) last_buttons[i] = current_buttons[i];

  // Triangle (index 5) -> manual UNLOCK
  if (press_list[5] == 1 && state == STATE_AT_SHOT_LOCKED) {
    Serial.println("TRIANGLE pressed: manual UNLOCK attempt");
    if (clampClosed) {
      digitalWrite(lockPin, UNLOCKED);
      lockState = UNLOCKED;
      state = STATE_UNLOCKED;
      Serial.println("Manual UNLOCK: lock set to UNLOCKED");
    } else {
      Serial.println("Cannot UNLOCK: clamp not closed.");
    }
  }

  // Square -> arm by square (distance)
  if (press_list[6] == 1 && state == STATE_AT_SHOT_LOCKED) {
    if (clampClosed) {
      armedBySquare = true;
      arm_timestamp = millis();
      Serial.println("SQUARE: armedBySquare = true");
    } else {
      Serial.println("SQUARE pressed but clamp not closed -> ignored");
    }
  }

  // Up -> arm by Up
  if (press_list[1] == 1 && state == STATE_AT_SHOT_LOCKED) {
    if (clampClosed) {
      armedByUp = true;
      arm_timestamp = millis();
      Serial.println("UP: armedByUp = true");
      PS4.setRumble(100, 100); PS4.setLed(0,255,0); PS4.sendToController();
      delay(80);
      PS4.setRumble(0,0); PS4.setLed(255,0,0); PS4.sendToController();
    } else {
      Serial.println("UP pressed but clamp not closed -> ignored");
    }
  }

  // Circle -> manual clamp toggle (safe-guarded)
  if (press_list[4] == 1) {
    if (state == STATE_UNLOCKED || state == STATE_MOVING_TO_SHOT || state == STATE_RETURNING_VIA_SENSOR) {
      Serial.println("Cannot toggle clamp during unlocked/motion states.");
    } else {
      clampClosed = !clampClosed;
      digitalWrite(clampPin, clampClosed ? CLAMP_CLOSED_LEVEL : CLAMP_OPEN_LEVEL);
      Serial.print("CIRCLE: clamp toggled -> closed? "); Serial.println(clampClosed ? "YES":"NO");
    }
  }
}

// ---------- CAN TX task ----------
void task_cantx(void* param) {
  for (;;) {
    send_rm_frame();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// ---------- send RM frame (CAN0) ----------
void send_rm_frame() {
  CAN_FRAME tx_msg0;
  CAN_FRAME tx_msg1;
  tx_msg0.id = 0x200;
  tx_msg0.length = 8;
  tx_msg1.id = 0x1FF;
  tx_msg1.length = 8;
  for (int i = 0; i < 8; i++) {
    tx_msg0.data.byte[i] = motors.can_msg0[i];
    tx_msg1.data.byte[i] = motors.can_msg1[i];
  }
  CAN0.sendFrame(tx_msg0);
  CAN0.sendFrame(tx_msg1);
}

// ---------- CAN RX callback ----------
void can0_callback(CAN_FRAME *frame) {
  int rx_id = frame->id;
  if (rx_id > 0x200 && rx_id < 0x209) {
    motors.update_motor_status(rx_id - 0x201, micros(),
      frame->data.uint8[0] << 8 | frame->data.uint8[1],
      frame->data.uint8[2] << 8 | frame->data.uint8[3]);
  }

  if (homing_stage == 1) {
    motors.set_target_rpm(4, 1000);
  }

  if (initial_stage == 0) {
    motors.set_target_rpm(4, 0);
    initial_encoder = motors.gearbox_pos[4];
    initial_stage = 1;
    shot_requested = true;
    home_start_time = millis();
    digitalWrite(lockPin, LOCKED);
    lockState = LOCKED;
    state = STATE_HOMED_LOCKED;
    Serial.printf("HOMED: initial_encoder=%ld\n", initial_encoder);
  }
}

// ---------- shooter logic ----------
void shooter_logic(float distance_local) {
  if (shot_requested && !shot_in_progress && (millis() - home_start_time >= homed_Delay)) {
    shot_target_pos = initial_encoder + top;
    motors.set_target_pos(4, shot_target_pos);
    shot_in_progress = true;
    shot_requested = false;
    state = STATE_MOVING_TO_SHOT;
    Serial.printf("AUTO: moving to shot pos=%ld\n", shot_target_pos);
  }

  if (state == STATE_MOVING_TO_SHOT) {
    long cur = motors.gearbox_pos[4];
    long diff = (cur > shot_target_pos) ? (cur - shot_target_pos) : (shot_target_pos - cur);
    if (diff <= POSITION_TOLERANCE) {
      shot_in_progress = false;
      state = STATE_AT_SHOT_LOCKED;
      Serial.println("Arrived at shot position -> AT_SHOT_LOCKED");
    }
  }

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

void find_home(){
  homing_stage = 1;
  state = STATE_HOMING;
  Serial.println("Setup complete. Starting homing...");
}

// ---------- clamp logic (mot idx 5) ----------
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

// ---------- setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(sensorPin, INPUT);
  pinMode(lockPin, OUTPUT);
  pinMode(clampPin, OUTPUT);

  digitalWrite(lockPin, UNLOCKED);
  lockState = UNLOCKED;
  digitalWrite(clampPin, CLAMP_OPEN_LEVEL); // 使用新的 CLAMP_OPEN_LEVEL (LOW)
  clampClosed = false;

  Wire.begin(thisAddress);
  Wire.onReceive(receiveEvent);

  PS4.begin("a0:a0:a0:a0:a0:a0");
  PS4.attach(notify);
  PS4.attachOnConnect([](){ Serial.println("PS4 connected"); });
  PS4.attachOnDisconnect([](){ Serial.println("PS4 disconnected"); });

  // init motors
  for (int i = 0; i < motors.MOTOR_NUM; i++) {
    motors.set_pid_rpm(i, 6, 0.01, 0);
    motors.reset_gearbox_pos(i);
  }

  // CAN0 init (single CAN) - 直接啟動並註冊監聽與 callback（不檢查回傳值）
  CAN0.begin(1000000);
  CAN0.watchFor();
  CAN0.setGeneralCallback(can0_callback);

  attachInterrupt(sensorPin, sensorISR, FALLING);

  find_hone();

  // start CAN TX task
  xTaskCreatePinnedToCore(task_cantx, "cantx", 2048, NULL, 1, NULL, 1);
  delay(50);
  int_pos = motors.gearbox_pos[5];
}

// ---------- loop ----------
void loop() {
  static float distance_local = 0.0f;
  if (newRxData) {
    noInterrupts();
    distance_local = rxData.Distance;
    newRxData = false;
    interrupts();
  }
  shooter_logic(distance_local);
  clamp_logic();

  if (millis() - last_time > 250) {
    last_time = millis();
    Serial.printf("STATE=%d shooterPos=%ld clampPos=%ld clampClosed=%d armedUp=%d armedSq=%d lock=%d\n",
      (int)state, motors.gearbox_pos[4], motors.gearbox_pos[5], (int)clampClosed, (int)armedByUp, (int)armedBySquare, digitalRead(lockPin));
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);
}
