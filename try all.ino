// Full integrated code: Wheels (CAN0) + Shooter & Clamp (CAN1) + PS4
#include <Arduino.h>
#include "MecanumWheelDriver.h"
#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>

// ---------------- CAN0 (Wheels) ----------------
#define MOTOR_NUM 4
MecanumWheelDriver motor_driver(-1.0, -1.0, -1.0);
int rm_speed[MOTOR_NUM], rm_speed_error[MOTOR_NUM], rm_last_speed_error[MOTOR_NUM];
int rm_target_speed[MOTOR_NUM], rm_output[MOTOR_NUM];
double rm_sk[MOTOR_NUM][3] = {
  {1.5, 0.0, 2.75}, {1.5, 0.0, 2.75}, {1.5, 0.0, 2.75}, {1.5, 0.0, 2.75}
};
double rm_so[MOTOR_NUM][3] = {0};
const int MAX_OUTPUT = 16384;
const double MAX_SO = 1000.0;

CAN_FRAME tx_msg, joy_feedback;
extern ESP32CAN CAN0;
bool CAN0_OK = false;

// ---------------- CAN1 (Shooter + Reloader + Clamp) ----------------
#define lockPin   12   // 電磁鎖
#define clampPin  13   // 夾爪輸出
#define sensorPin 32   // homing 感測器
#define LOCKED LOW
#define UNLOCKED HIGH
#define CLAMP_CLOSED_LEVEL LOW  // 夾爪 LOW 表示 closed（與你早先 clamp code 保持一致）
#define CLAMP_OPEN_LEVEL   HIGH

Rm_set motors;
extern ESP32CAN CAN1;
bool CAN1_OK = false;

// Shooter vars (使用 motors index 0 為 shooter)
volatile int homing_stage = -1, initial_stage = -1;
volatile long initial_encoder = 0;
volatile bool shot_requested = false;
bool shot_in_progress = false;
long shot_target_pos = 0;
bool lockState = UNLOCKED;

enum ShooterState { STATE_IDLE, STATE_HOMING, STATE_READY, STATE_SHOOTING, STATE_RETURNING };
volatile ShooterState shooter_state = STATE_IDLE;

// Reloader / clamp related (使用 motors index 1 為 reloader/升降)
long initial_pos_reload = 0, target_pos_reload = 0;
bool reload_in_progress = false, reload_done = false;
int reload_step = 0;
unsigned long step_timer = 0;
const long RELOAD_OFFSET = 20000;

// Clamp state (shared)
volatile bool clampClosed = false; // true = closed (CLAMP_CLOSED_LEVEL)
unsigned long clamp_last_toggle = 0;

// PS4
int press_list[16];
bool current_buttons[16], last_buttons[16];
bool is_PS4_connected = false;
float linear_x = 0, linear_y = 0, angular_z = 0;

// Armed by Up
volatile bool armedByUp = false;
unsigned long arm_timestamp = 0;
const unsigned long ARM_TIMEOUT = 5000UL; // 5 秒超時

// Helpers
float restrictRange(float x, float min, float max) { return x < min ? min : (x > max ? max : x); }

void PIDSpeedCalculate(int id) {
  rm_last_speed_error[id] = rm_speed_error[id];
  rm_speed_error[id] = rm_target_speed[id] - rm_speed[id];
  if (rm_speed[id] > 32768) rm_speed_error[id] += 65536;
  rm_so[id][0] = rm_sk[id][0] * rm_speed_error[id];
  rm_so[id][1] += rm_speed_error[id];
  rm_so[id][1] = restrictRange(rm_so[id][1], -MAX_SO, MAX_SO);
  rm_so[id][2] = rm_sk[id][2] * (rm_speed_error[id] - rm_last_speed_error[id]);
  rm_output[id] = rm_so[id][0] + rm_so[id][1] * rm_sk[id][1] + rm_so[id][2];
  rm_output[id] = restrictRange(rm_output[id], -MAX_OUTPUT, MAX_OUTPUT);
}

// ---------------- CAN0 (Wheels) ----------------
void sendCAN0Frame() {
  if (!CAN0_OK) return;
  tx_msg.id = 0x200;
  tx_msg.length = 8;
  for (int i = 0; i < MOTOR_NUM; i++) {
    tx_msg.data.byte[2*i] = (rm_output[i] >> 8) & 0xFF;
    tx_msg.data.byte[2*i+1] = rm_output[i] & 0xFF;
  }
  CAN0.sendFrame(tx_msg);
}

void sendJoystickFeedback() {
  if (!CAN0_OK) return;
  joy_feedback.id = 0xF9;
  joy_feedback.length = 8;
  joy_feedback.data.byte[0] = min(PS4.LStickX() + 128, 255);
  joy_feedback.data.byte[1] = min(PS4.LStickY() + 128, 255);
  joy_feedback.data.byte[2] = min(PS4.RStickX() + 128, 255);
  joy_feedback.data.byte[3] = min(PS4.RStickY() + 128, 255);
  for (int i=4;i<8;i++) joy_feedback.data.byte[i] = 0;
  CAN0.sendFrame(joy_feedback);
}

void can0_callback(CAN_FRAME *frame) {
  if (frame->id >= 0x201 && frame->id < 0x201 + MOTOR_NUM) {
    int id = frame->id - 0x201;
    rm_speed[id] = (frame->data.byte[2] << 8) | frame->data.byte[3];
    PIDSpeedCalculate(id);
    sendCAN0Frame();
  }
}

// ---------------- CAN1 (Shooter + Clamp) ----------------
void send_rm_frame() {
  if (!CAN1_OK) return;
  CAN_FRAME tx;
  tx.id = 0x200;
  tx.length = 8;
  for (int i=0; i<8; i++) tx.data.byte[i] = motors.can_msg0[i];
  CAN1.sendFrame(tx);
}

void can1_callback(CAN_FRAME *frame) {
  if (!CAN1_OK) return;
  int rx_id = frame->id;
  // 更新 motor status（假設來自 0x201 起）
  motors.update_motor_status(rx_id - 0x201, micros(),
    frame->data.uint8[0] << 8 | frame->data.uint8[1],
    frame->data.uint8[2] << 8 | frame->data.uint8[3]);

  // Homing: 若 homing_stage == 1 就讓 shooter motor 轉向感測器
  if (homing_stage == 1) {
    motors.set_target_rpm(0, 1000); // 根據需要調整 rpm
  }

  // ISR 標記 sensor hit 後 initial_stage==0 在此處 latch encoder
  if (initial_stage == 0) {
    motors.set_target_rpm(0, 0);
    initial_encoder = motors.gearbox_pos[0];
    initial_stage = 1;
    digitalWrite(lockPin, LOCKED);
    lockState = LOCKED;
    shooter_state = STATE_READY;
    Serial.printf("Shooter homed: initial_encoder=%ld\n", initial_encoder);
  }
}

// 介於 reloader 與 clamp 的簡單 state machine（基於你原本邏輯）
void startReload() {
  reload_in_progress = true; reload_step = 0; step_timer = millis(); reload_done = false;
  Serial.println("Reload started");
}
void reloaderLogic() {
  if (!reload_in_progress) return;
  unsigned long now = millis();
  switch(reload_step) {
    case 0:
      motors.set_target_pos(1, target_pos_reload);
      if (abs(motors.gearbox_pos[1]-target_pos_reload)<500) { reload_step++; step_timer=now; }
      break;
    case 1:
      digitalWrite(clampPin, CLAMP_CLOSED_LEVEL); // 關夾
      clampClosed = true;
      if (now-step_timer>300){ reload_step++; step_timer=now; }
      break;
    case 2:
      digitalWrite(clampPin, CLAMP_OPEN_LEVEL); // 開夾
      clampClosed = false;
      if (now-step_timer>300){ reload_step++; step_timer=now; }
      break;
    case 3:
      digitalWrite(clampPin, CLAMP_CLOSED_LEVEL);
      clampClosed = true;
      if (now-step_timer>300){ reload_step++; step_timer=now; }
      break;
    case 4:
      digitalWrite(clampPin, CLAMP_OPEN_LEVEL);
      clampClosed = false;
      if (now-step_timer>300){ reload_step++; step_timer=now; }
      break;
    case 5:
      motors.set_target_pos(1, initial_pos_reload);
      if (abs(motors.gearbox_pos[1]-initial_pos_reload)<500) { reload_step++; }
      break;
    case 6:
      reload_in_progress=false; reload_done=true; Serial.println("Reload done");
      break;
  }
}

// Shooter logic (包含 Up -> armedByUp)
void shooterLogic() {
  // Triangle 發射：只有在 shooter ready 且已經上膛（armedByUp）或直接允許時才發射
  // 我們保留原有直接按 Triangle 發射的機制，但若加入 armedByUp，Triangle 允許在上膛後觸發
  if (press_list[5] == 1) { // Triangle rising edge
    if (shooter_state == STATE_READY) {
      // 若使用上膛機制，需先 armedByUp 才能發射；但為了兼容性，我們讓 Triangle 直接發射也可（取消註解則需要 arm）
      // // require armed:
      // if (!armedByUp) { Serial.println("Not armed! Press Up first."); }
      // else { shot_requested = true; }
      shot_requested = true;
      // 若你希望 Triangle 僅在 arm 後才可發射，改成上面被註解的檢查邏輯
      Serial.println("Triangle pressed -> shot requested");
    }
  }

  // 新增: 若 Up 已經上膛 (armedByUp) 且有超時，清除
  if (armedByUp && millis() - arm_timestamp > ARM_TIMEOUT) {
    armedByUp = false;
    Serial.println("Arm timed out (armedByUp cleared)");
  }

  // 處理 shot request
  if (shot_requested && !shot_in_progress) {
    // 在發射前做最後安全檢查：clamp 必須為 closed、lockState 必須為 LOCKED
    if (!clampClosed) {
      Serial.println("Abort shot: clamp not closed");
      shot_requested = false;
      return;
    }
    if (lockState != LOCKED) {
      Serial.println("Abort shot: lock not locked");
      shot_requested = false;
      return;
    }
    // 啟動發射動作，這裡用初始 encoder - 400000 作為例子（與你原來一致）
    shot_target_pos = initial_encoder - 400000;
    motors.set_target_pos(0, shot_target_pos);
    shot_in_progress = true; shot_requested = false; shooter_state = STATE_SHOOTING;
    Serial.printf("Shot started -> target=%ld\n", shot_target_pos);
  }

  // 當到達射擊終點後，回到初始位置
  if (shooter_state == STATE_SHOOTING &&
      abs(motors.gearbox_pos[0] - shot_target_pos) < 1000) {
    shooter_state = STATE_RETURNING;
    motors.set_target_pos(0, initial_encoder);
    Serial.println("Shot reached, returning to home");
  }
  if (shooter_state == STATE_RETURNING &&
      abs(motors.gearbox_pos[0] - initial_encoder) < 1000) {
    shooter_state = STATE_READY; shot_in_progress = false;
    Serial.println("Shooter ready again");
  }
}

// ---------------- PS4 callback ----------------
void notify() {
  // 只讀我們需要的按鈕（為簡潔起見）
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

  for (int i=0;i<16;i++) {
    press_list[i] = (int)current_buttons[i] - (int)last_buttons[i];
    last_buttons[i] = current_buttons[i];
  }

  // Up (index 1) => Arm
  if (press_list[1] == 1) { // rising edge
    if (shooter_state == STATE_READY && clampClosed) {
      armedByUp = true;
      arm_timestamp = millis();
      Serial.println("Armed by UP");
      // 可做 PS4 LED 或震動提示
      PS4.setRumble(100, 100); PS4.setLed(0,255,0); PS4.sendToController();
      delay(80);
      PS4.setRumble(0,0); PS4.setLed(255,0,0); PS4.sendToController();
    } else {
      Serial.println("Cannot arm: shooter not ready or clamp not closed");
    }
  }

  // Cross (index 7) => toggle clamp 手動切換
  if (press_list[7] == 1) {
    // 若 shooter 處於 UNLOCK 或正在 shooting，禁止切換夾爪（互鎖）
    if (shooter_state == STATE_SHOOTING || shooter_state == STATE_RETURNING) {
      Serial.println("Cannot toggle clamp during shooting/returning");
    } else {
      clampClosed = !clampClosed;
      digitalWrite(clampPin, clampClosed ? CLAMP_CLOSED_LEVEL : CLAMP_OPEN_LEVEL);
      Serial.print("Clamp toggled manually -> closed? "); Serial.println(clampClosed ? "YES":"NO");
      clamp_last_toggle = millis();
    }
  }

  // Square (index 6) => start reload（若未在 reload 中）
  if (press_list[6] == 1) {
    if (!reload_in_progress) startReload();
  }

  // Triangle (index 5) 的 rising edge 會在 shooterLogic 處理 shot request
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT);
  pinMode(lockPin, OUTPUT);
  pinMode(clampPin, OUTPUT);
  digitalWrite(lockPin, UNLOCKED);          // 開機解鎖狀態
  digitalWrite(clampPin, CLAMP_OPEN_LEVEL); // 夾爪預設開啟
  clampClosed = false;
  lockState = UNLOCKED;

  PS4.begin("01:02:03:04:05:06");
  PS4.attach(notify);

  // CAN0 (wheels)
  CAN0_OK = CAN0.begin(1000000);
  if (CAN0_OK) {
    for (int i=0;i<MOTOR_NUM;i++) CAN0.watchFor(0x201+i);
    CAN0.setGeneralCallback(can0_callback);
    Serial.println("CAN0 started");
  } else { Serial.println("CAN0 failed to start"); }

  // CAN1 (shooter + reloader + clamp)
  CAN1_OK = CAN1.begin(1000000);
  if (CAN1_OK) {
    CAN1.watchFor(); // 接收所有
    CAN1.setGeneralCallback(can1_callback);
    Serial.println("CAN1 started");
  } else { Serial.println("CAN1 failed to start"); }

  // init motors (設定 PID 並 reset gearbox pos)
  for (int i = 0; i < motors.MOTOR_NUM; i++) {
    motors.set_pid_rpm(i, 6, 0.01, 0);
    motors.reset_gearbox_pos(i);
  }

  // 初始化 reloader 位置
  initial_pos_reload = motors.gearbox_pos[1];
  target_pos_reload = initial_pos_reload + RELOAD_OFFSET;

  // attach sensor interrupt（匿名 lambda）
  attachInterrupt(digitalPinToInterrupt(sensorPin), [](){
    if (homing_stage > 0) { homing_stage = 0; initial_stage = 0; }
  }, FALLING);

  // 啟動 shooter homing
  homing_stage = 1;
  shooter_state = STATE_HOMING;
  Serial.println("Setup complete. Shooter homing started.");
}

// ---------------- Loop ----------------
void loop() {
  // 若 PS4 未連線，短暫等待
  if (!PS4.isConnected()) { delay(50); return; }

  // CAN0: joystick 控制與速度計算（輪子）
  linear_x = PS4.LStickX()/127.0;
  linear_y = PS4.LStickY()/127.0;
  angular_z = PS4.RStickX()/127.0;
  motor_driver.getMovement(rm_target_speed, -linear_x, linear_y, -angular_z);
  sendJoystickFeedback();

  // PS4 按鍵: reload 或 manual clamp handled in notify()
  if (press_list[6] == 1 && !reload_in_progress) startReload();

  // 如果按住 Cross (實作：長按直接夾住，這裡保留單次 toggle 行為)
  // if (PS4.Cross()) digitalWrite(clampPin, CLAMP_CLOSED_LEVEL);

  // 主要控制邏輯
  reloaderLogic();
  shooterLogic();

  // 定期送 CAN1 frame（驅動 RM）
  send_rm_frame();

  // 若需要也可每回合送 CAN0 控制訊息
  // sendCAN0Frame(); // 在 can0_callback 已作呼叫

  delay(10);
}
