#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>

// ===================== PINS & CONSTANTS =====================
#define CLAMP_PIN              14          // clamp: LOW=closed, HIGH=open (your wiring)
#define LOCK_PIN               13          // shooter lock: LOW=LOCKED, HIGH=UNLOCKED
#define SENSOR_PIN             32          // shooter homing sensor (FALLING edge)

#define POSITION_TOLERANCE     500L
#define JOYSTICK_DEADZONE      12
#define USE_RPM_FALLBACK       1           // 0: rely only on position controller

// Shooter timing
const unsigned long HOMED_DELAY_MS       = 2000UL;
const unsigned long UNLOCK_AUTO_TIMEOUT  = 2000UL;

// Shooter kinematics (motor 4)
int    shot_offset_top = -400000;         // target = initial_encoder + shot_offset_top

// Arm (motor 5) positions (from its latched home)
int pos_A = 130000;
int pos_B =  90000;

// Macros for shooter lock
#define LOCKED    LOW
#define UNLOCKED  HIGH

// ===================== GLOBALS =====================
Rm_set motors;

// ---- PS4 / buttons
int lx, ly, rx, ry;
bool current_buttons[16] = {0};
bool last_buttons[16]    = {0};
int  press_list[16]      = {0};

// ---- Shooter state (motor 4 + sensor + lock)
volatile int  homing_stage     = -1;
volatile int  initial_stage    = -1;
volatile long initial_encoder4 = 0;  // latched on sensor hit
volatile bool shot_requested   = false;
bool           shot_in_progress= false;
bool           returning_via_sensor = false;

enum MachineState {
  STATE_HOMING,
  STATE_HOMED_LOCKED,
  STATE_MOVING_TO_SHOT,
  STATE_AT_SHOT_LOCKED,
  STATE_UNLOCKED,
  STATE_RETURNING_HOME,        // (kept for completeness; not used directly)
  STATE_RETURNING_VIA_SENSOR
};
volatile MachineState state = STATE_HOMING;

bool lockState = UNLOCKED;   // GPIO level we’re driving
unsigned long home_start_time = 0;
unsigned long unlock_time     = 0;
long          shot_target_pos = 0;

// ---- Clamp + Arm (motor 5 without sensor)
volatile bool clampClosed     = false;  // Circle toggles
volatile bool homeLatched5    = false;  // true when first valid feedback seen
long          int_pos5        = 0;      // arm “home” latched once from CAN (motor 5)
long          current_pos5    = 0;      // live pos (motor 5)
bool          arm_went_up     = false;  // gating DOWN after UP

// ---- Debug tick
unsigned long last_tick_us = 0UL;

// ===================== FORWARD DECLS =====================
void send_rm_frame();
void drive_toward_target_by_rpm(long target);

// ===================== PS4 CALLBACK =====================
void notify() {
  // Sticks
  lx = PS4.LStickX();   ly = PS4.LStickY();
  rx = PS4.RStickX();   ry = PS4.RStickY();
  if (abs(lx) < JOYSTICK_DEADZONE) lx = 0;
  if (abs(ly) < JOYSTICK_DEADZONE) ly = 0;
  if (abs(rx) < JOYSTICK_DEADZONE) rx = 0;
  if (abs(ry) < JOYSTICK_DEADZONE) ry = 0;

  // Buttons
  current_buttons[0]  = PS4.Right();     current_buttons[1]  = PS4.Up();
  current_buttons[2]  = PS4.Left();      current_buttons[3]  = PS4.Down();
  current_buttons[4]  = PS4.Circle();    current_buttons[5]  = PS4.Triangle();
  current_buttons[6]  = PS4.Square();    current_buttons[7]  = PS4.Cross();
  current_buttons[8]  = PS4.L1();        current_buttons[9]  = PS4.R1();
  current_buttons[10] = PS4.L2();        current_buttons[11] = PS4.R2();
  current_buttons[12] = PS4.Options();   current_buttons[13] = PS4.Share();
  current_buttons[14] = PS4.PSButton();  current_buttons[15] = PS4.Touchpad();

  for (int i = 0; i < 16; i++) press_list[i] = (int)current_buttons[i] - (int)last_buttons[i];
  for (int i = 0; i < 16; i++) last_buttons[i]  = current_buttons[i];

  // ---- Clamp toggle (CIRCLE) ----
  if (press_list[4] == 1) {
    clampClosed = !clampClosed;
    Serial.printf("[Clamp] toggled. Closed? %s\n", clampClosed ? "YES" : "NO");
  }

  // ---- Shooter unlock (TRIANGLE) ----
  if (press_list[5] == 1 && state == STATE_AT_SHOT_LOCKED) {
    Serial.println("[Shooter] TRIANGLE -> UNLOCK");
    lockState = UNLOCKED;
    digitalWrite(LOCK_PIN, lockState);
    state = STATE_UNLOCKED;
    unlock_time = millis();
    // optional: release PID so it won't fight
    motors.set_target_rpm(4, 0);
  }

  // ---- Arm UP (motor 5) ----
  if (press_list[1] == 1) {
    if (!homeLatched5) {
      Serial.println("[Arm] UP ignored: motor5 home not latched yet.");
    } else if (!clampClosed) {
      Serial.println("[Arm] UP ignored: clamp open; close first.");
    } else {
      long target_A = int_pos5 + pos_A;
      Serial.printf("[Arm] UP -> target=%ld (home=%ld cur=%ld)\n", target_A, int_pos5, current_pos5);
      motors.set_target_rpm(5, 0);
      motors.set_target_pos(5, target_A);
      send_rm_frame();
      arm_went_up = true;
    }
  }

  // ---- Arm DOWN (motor 5) ----
  if (press_list[3] == 1 && clampClosed ) {
    Serial.printf("[Arm] DOWN press: up=%d home5=%d cur5=%ld\n", arm_went_up, (int)homeLatched5, current_pos5);
    if (!homeLatched5) {
      Serial.println("[Arm] DOWN ignored: motor5 home not latched.");
    } else {
      bool allow_down = arm_went_up || (current_pos5 > (int_pos5 + pos_B/2));
      if (!allow_down) {
        Serial.println("[Arm] DOWN ignored: arm not up enough.");
      } else {
        long target_B = int_pos5 + pos_B;
        Serial.printf("[Arm] DOWN -> target=%ld (home=%ld cur=%ld)\n", target_B, int_pos5, current_pos5);
        motors.set_target_rpm(5, 0);
        motors.set_target_pos(5, target_B);
        send_rm_frame();
        arm_went_up = false;
      }
    }
  }
}

// ===================== PS4 CONNECT EVENTS =====================
void onConnect() {
  PS4.setLed(255, 0, 0);
  PS4.sendToController();
  Serial.println("PS4 connected");
}
void onDisconnect() {
  Serial.println("PS4 disconnected");
}

// ===================== SENSOR ISR (shooter homing) =====================
void IRAM_ATTR SENSOR_ISR() {
  // Sensor falling -> mark that homing sensor was hit
  if (homing_stage > 0) {
    homing_stage  = 0;
    initial_stage = 0;  // will be processed in CAN callback
  }
}

// ===================== SHOOTER HELPERS =====================
void find_home() {
  homing_stage = 1;
  state = STATE_HOMING;
  Serial.println("[Shooter] find_home() -> homing_stage=1");
}

void start_return_via_sensor() {
  if (state == STATE_RETURNING_VIA_SENSOR || returning_via_sensor) return;
  Serial.println("[Shooter] Starting return via sensor.");
  // Ensure unlocked
  digitalWrite(LOCK_PIN, UNLOCKED);
  lockState = UNLOCKED;
  delay(10);
  returning_via_sensor = true;
  state = STATE_RETURNING_VIA_SENSOR;
  find_home(); // CAN callback will drive motor 4 toward sensor
}

void drive_toward_target_by_rpm(long target) {
  long cur  = motors.gearbox_pos[4];
  long diff = labs(cur - target);
  if (diff <= POSITION_TOLERANCE) {
    motors.set_target_rpm(4, 0);
    shot_in_progress = false;
    state = STATE_AT_SHOT_LOCKED;
    Serial.println("[Shooter] Arrived at shot target (via rpm fallback).");
    return;
  }
  int dir = (cur < target) ? 1 : -1;
  int rpm = 1200 + (int)(diff / 1000);
  if (rpm > 3000) rpm = 3000;
  motors.set_target_rpm(4, dir * rpm);
}

// ===================== CAN TX TASK =====================
void task_cantx(void* param) {
  for (;;) {
    send_rm_frame();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ===================== CAN SEND =====================
void send_rm_frame() {
  // RM IDs 1-4 -> 0x200 ; 5-8 -> 0x1FF (per your library)
  CAN_FRAME tx0, tx1;
  tx0.id = 0x200; tx0.length = 8;
  tx1.id = 0x1FF; tx1.length = 8;
  for (int i = 0; i < 8; i++) {
    tx0.data.byte[i] = motors.can_msg0[i];
    tx1.data.byte[i] = motors.can_msg1[i];
  }
  CAN0.sendFrame(tx0);
  CAN0.sendFrame(tx1);
}

// ===================== CAN RX CALLBACK =====================
void can0_callback(CAN_FRAME *frame) {
  int rx_id = frame->id;
  if (rx_id > 0x200 && rx_id < 0x209) {
    motors.update_motor_status(
      rx_id - 0x201,
      micros(),
      (frame->data.uint8[0] << 8) | frame->data.uint8[1],
      (frame->data.uint8[2] << 8) | frame->data.uint8[3]
    );
  }

  // Keep live positions for both motors we care about
  long cur4 = motors.gearbox_pos[4];  // shooter axis
  current_pos5 = motors.gearbox_pos[5];

  // Latch arm (motor 5) home once on first valid feedback
  if (!homeLatched5 && current_pos5 != 0) {
    int_pos5 = current_pos5;
    homeLatched5 = true;
    Serial.printf("[Arm] Home latched from CAN: home5=%ld\n", int_pos5);
  }

  // Shooter homing drive while homing_stage==1
  if (homing_stage == 1) {
    motors.set_target_rpm(4, 1000); // drive toward sensor (tune as needed)
  }

  // On sensor hit (initial_stage==0), stop, latch, lock
  if (initial_stage == 0) {
    motors.set_target_rpm(4, 0);
    initial_encoder4 = cur4;  // latch shooter home
    initial_stage = 1;
    shot_requested = true;     // preserve your original behaviour
    home_start_time = millis();

    // Lock after homing
    digitalWrite(LOCK_PIN, LOCKED);
    lockState = LOCKED;

    if (returning_via_sensor) {
      returning_via_sensor = false;
      state = STATE_HOMED_LOCKED;
      Serial.printf("[Shooter] RETURN VIA SENSOR: initial_encoder4=%ld -> LOCKED\n", initial_encoder4);
    } else {
      state = STATE_HOMED_LOCKED;
      Serial.printf("[Shooter] HOMED: initial_encoder4=%ld -> LOCKED\n", initial_encoder4);
    }
  }
}

// ===================== SETUP =====================
void setup() {
  // IO
  pinMode(SENSOR_PIN, INPUT);
  pinMode(LOCK_PIN, OUTPUT);
  pinMode(CLAMP_PIN, OUTPUT);

  // Start with shooter unlocked; homing callback will lock
  digitalWrite(LOCK_PIN, UNLOCKED);
  lockState = UNLOCKED;

  // Clamp starts open (HIGH), your original logic
  digitalWrite(CLAMP_PIN, HIGH);
  clampClosed = false;

  attachInterrupt(SENSOR_PIN, SENSOR_ISR, FALLING);

  // Serial & PS4
  Serial.begin(115200);
  PS4.begin("a0:a0:a0:a0:a0:a0");
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);

  // Motors
  for (int i = 0; i < motors.MOTOR_NUM; i++) {
    motors.set_pid_rpm(i, 6, 0.01, 0);
    motors.reset_gearbox_pos(i);
  }
  // If position PID is available and tuned, you can enable for motor 4:
  // motors.set_pid_pos(4, 6, 0.01, 0);

  // CAN
  CAN0.begin(1000000);
  CAN0.watchFor();
  CAN0.setGeneralCallback(can0_callback);

  // CAN TX task
  xTaskCreatePinnedToCore(task_cantx, "cantx", 2048, NULL, 1, NULL, 1);

  // Initial homing for shooter
  find_home();

  Serial.println("Setup complete. Waiting for first CAN feedback...");
}

// ===================== LOOP =====================
void loop() {
  // Reflect clamp GPIO continuously
  digitalWrite(CLAMP_PIN, clampClosed ? LOW : HIGH);

  // Main tick ~20ms
  if (micros() - last_tick_us > 20000) {
    last_tick_us = micros();

    // Shooter state machine maintenance
    long cur4 = motors.gearbox_pos[4];
    Serial.printf("STATE=%d SHOOT_CUR=%ld SHOOT_HOME=%ld LOCK=%d clampClosed=%d ARM_CUR5=%ld ARM_HOME5=%ld\n",
                  (int)state, cur4, initial_encoder4, digitalRead(LOCK_PIN),
                  (int)clampClosed, current_pos5, int_pos5);

    // Auto move to shot after homed
    if (shot_requested && !shot_in_progress && (millis() - home_start_time >= HOMED_DELAY_MS)) {
      shot_target_pos = initial_encoder4 + shot_offset_top;
      motors.set_target_pos(4, shot_target_pos);
      shot_in_progress = true;
      shot_requested   = false;
      state = STATE_MOVING_TO_SHOT;
      Serial.printf("[Shooter] AUTO move to shot: target=%ld\n", shot_target_pos);
    }

    // Monitor moving-to-shot arrival
    if (state == STATE_MOVING_TO_SHOT) {
      long diff = labs(motors.gearbox_pos[4] - shot_target_pos);
      Serial.printf("[Shooter] MOVING: cur=%ld target=%ld diff=%ld\n", motors.gearbox_pos[4], shot_target_pos, diff);
      if (diff <= POSITION_TOLERANCE) {
        motors.set_target_rpm(4, 0);
        shot_in_progress = false;
        state = STATE_AT_SHOT_LOCKED;
        Serial.println("[Shooter] Arrived at shot (locked).");
      } else {
#if USE_RPM_FALLBACK
        drive_toward_target_by_rpm(shot_target_pos);
#endif
      }
    }

    // AUTO: if unlocked for >= 2s, auto return via sensor
    if (state == STATE_UNLOCKED) {
      if ((millis() - unlock_time >= UNLOCK_AUTO_TIMEOUT) && !returning_via_sensor) {
        Serial.println("[Shooter] UNLOCK timeout -> auto return via sensor.");
        start_return_via_sensor();
      }
    }
  }
}
