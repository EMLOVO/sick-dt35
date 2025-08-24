#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>

#define clamp 14
#define JOYSTICK_DEADZONE 12

Rm_set motors;
long int_pos = 0;
long current_pos = 0;
int lx, ly, rx, ry;
bool up = false;

unsigned long stateTimer = 0;
int shakeCounter = 0;      
int shakeDelay = 100;       

bool current_buttons[16];
bool last_buttons[16];
int press_list[16];

volatile bool clampClosed = false;

int pos_A = 120000;
int pos_B = 90000;

volatile bool homeLatched = false;

// ===== 新增：狀態機枚舉 =====
enum AutoState { IDLE, MOVING_UP,CLAMP_SHAKE, MOVING_DOWN, DONE };
AutoState autoState = IDLE;

unsigned long stateTimer = 0;  // 計時用

// ==================================
void notify()
{
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

  current_buttons[4] = PS4.Circle();   // toggle clamp
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
    press_list[i] = current_buttons[i] - last_buttons[i];
  }
  for (int i = 0; i < 16; i++) last_buttons[i] = current_buttons[i];

  // Circle: 手動測試夾爪
  if (press_list[4] == 1) {
    clampClosed = !clampClosed;
    Serial.print("Clamp toggled manually. Now closed? ");
    Serial.println(clampClosed ? "YES" : "NO");
  }

  // Up: 啟動自動流程
  if (press_list[1] == 1 && clampClosed && autoState == IDLE) {
    Serial.println("Auto UP sequence start!");
    motors.set_target_pos(5, int_pos + pos_A);
    autoState = MOVING_UP;
  }

  // Down: 隨時下降到 pos_B
  if (press_list[3] == 1) {
    Serial.println("Manual DOWN triggered!");
    motors.set_target_pos(5, int_pos);
    autoState = IDLE;  // 中斷自動流程
  }
}
void onConnect() {
  PS4.setLed(255, 0, 0);
  PS4.sendToController();
  Serial.println("PS4 connected");
}
void onDisconnect() {
  Serial.println("PS4 disconnected");
}

void send_rm_frame()
{
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
  CAN1.sendFrame(tx_msg0);
  CAN1.sendFrame(tx_msg1);
}

void can1_callback(CAN_FRAME *frame)
{
    // rm motor feedback
    int rx_id = frame->id;
    motors.update_motor_status(rx_id - 0x201,
                               micros(),
                               frame->data.uint8[0] << 8 | frame->data.uint8[1],
                               frame->data.uint8[2] << 8 | frame->data.uint8[3]);

    // keep a live copy
    current_pos = motors.gearbox_pos[5];

    // latch initial/home/reference position only once (first valid CAN feedback)
    if (!homeLatched) {
      int_pos = current_pos;
      homeLatched = true;
      Serial.print("Home latched once: ");
      Serial.println(int_pos);
    }
}

void setup()
{
    Serial.begin(115200);

    // set up CAN
    CAN1.begin(1000000);
    CAN1.watchFor();
    CAN1.setGeneralCallback(can1_callback);
    delay(50);

    // clamp pin
    pinMode(clamp, OUTPUT);
    digitalWrite(clamp, HIGH);
    clampClosed = false;

    // PS4
    PS4.begin("a0:a0:a0:a0:a0:a0");
    PS4.attach(notify);
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisconnect);

    // initialize motors properly (reset each motor index)
    for (int i = 0; i < motors.MOTOR_NUM; i++)
    {
        motors.set_pid_rpm(i, 6, 0.01, 0); // set pid for each rm motor
        motors.reset_gearbox_pos(i);       // <<< fixed: use index i
    }

    Serial.println("Setup done. Waiting for first CAN feedback to latch home position...");
}


// ==================================
void loop()
{
  // 狀態機控制流程
  switch (autoState) {
    case MOVING_UP:
      if (abs(current_pos - (int_pos + pos_A)) < 500) { // 到達上方
        Serial.println("Reached pos_A, opening clamp...");
        clampClosed = false;  // 自動開夾
        stateTimer = millis();
        autoState = WAIT_RELEASE;
      }
      break;

    case CLAMP_SHAKE:
      if (millis() - stateTimer > shakeDelay) {
        clampClosed = !clampClosed;   // 反轉狀態 (開→關→開→關)
        shakeCounter++;
        stateTimer = millis();
        if (shakeCounter >= 4) {      // 開關共做4次
          Serial.println("Shake done, moving down...");
          motors.set_target_pos(5, int_pos + pos_B);
          autoState = MOVING_DOWN;
        }
      }
      break;

    case MOVING_DOWN:
      if (abs(current_pos - (int_pos +)) < 500) { // 到達下方
        Serial.println("Sequence done, back to IDLE.");
        autoState = IDLE;
        shakeCounter = 0;
      }
      break;

    default:
      break;
  }

  // 反映夾爪輸出
  digitalWrite(clamp, clampClosed ? LOW : HIGH);

  // debug 輸出
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg >= 500) {
    Serial.print("live[5]=");
    Serial.print(current_pos);
    Serial.print("  latched_home=");
    Serial.print(int_pos);
    Serial.print("  clampClosed=");
    Serial.print(clampClosed);
    Serial.print("  autoState=");
    Serial.println(autoState);
    lastDbg = millis();
  }

  send_rm_frame(); // keep CAN frames flowing
}
