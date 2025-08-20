#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>

#define lock 13
#define sensor 32

const unsigned long return_Delay = 2000UL;
const unsigned long homed_Delay = 500UL;  
Rm_set motors;

volatile int homing_stage = -1;
volatile int initial_stage = -1;
volatile long initial_encoder = 0;    // written in CAN callback, read in loop

volatile bool shot_requested = false; // set in callback
bool shot_in_progress = false;       // handled in loop
bool shot_moving = false;            // moving to shot target
unsigned long shot_start_time = 0;
unsigned long home_start_time = 0;

volatile bool request_lock_high = false;
volatile bool request_lock_low  = false;

int one_round = -157318;

int last_time;

#define JOYSTICK_DEADZONE 12

int lx;  // left joystick x -128..128
int ly;  // left joystick y -128..128
int rx;  // right joystick x -128..128
int ry;  // right joystick y -128..128

bool shot_button_pressed = false;

bool current_buttons[16];
bool last_buttons[16];
int press_list[16];

void notify() {

  lx = (PS4.LStickX() );
  ly = (PS4.LStickY() );
  rx = (PS4.RStickX() );
  ry = (PS4.RStickY());

  if (abs(lx) < JOYSTICK_DEADZONE) { lx = 0; }
  if (abs(ly) < JOYSTICK_DEADZONE) { ly = 0; }
  if (abs(rx) < JOYSTICK_DEADZONE) { rx = 0; }
  if (abs(ry) < JOYSTICK_DEADZONE) { ry = 0; }

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
    press_list[i] = current_buttons[i] - last_buttons[i];
  }
  for (int i = 0; i < 16; i++) {
    last_buttons[i] = current_buttons[i];
  }

}

void onConnect() {
  PS4.setLed(255, 0, 0);  // Light up red when connected
  PS4.sendToController();
  Serial.println("Connected!");
}

void onDisconnect() {
  Serial.println("Disconnected!");
  // carbase.update_from_controller(0, 0, 0);
}

void IRAM_ATTR ISR() {
  if (homing_stage > 0) {
    homing_stage = 0;
    initial_stage = 0;
  }
}

void find_home() {
  homing_stage = 1;
}

void send_rm_frame() {
  CAN_FRAME tx_msg0;
  tx_msg0.id = 0x200;
  tx_msg0.length = 8;
  for (int i = 0; i < 8; i++) tx_msg0.data.byte[i] = motors.can_msg0[i];
  CAN1.sendFrame(tx_msg0);
}

void can1_callback(CAN_FRAME *frame)
{
  int rx_id = frame->id;
  motors.update_motor_status(rx_id - 0x201, micros(),
      frame->data.uint8[0] << 8 | frame->data.uint8[1],
      frame->data.uint8[2] << 8 | frame->data.uint8[3]);

  if (homing_stage == 1) {
    motors.set_target_rpm(0, 1000); // drive toward sensor
  }

  if (initial_stage == 0) {
    motors.set_target_rpm(0, 0);                // stop
    initial_encoder = motors.gearbox_pos[0];    // latch home
    initial_stage = 1;
    shot_requested = true;                      // request shoot from main loop
    // Serial.printf("Homed. encoder=%ld -> shot requested\n", initial_encoder);
  }
}

void setup()
{
  pinMode(sensor, INPUT);
  pinMode(lock, OUTPUT);
  digitalWrite(lock, LOW);
  Serial.begin(115200);

  PS4.begin("a0:a0:a0:a0:a0:a0");
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);

  for (int i = 0; i < motors.MOTOR_NUM; i++) {
    motors.set_pid_rpm(i, 6, 0.01, 0);
    motors.reset_gearbox_pos(i);
  }

  CAN1.begin(1000000);
  CAN1.watchFor();
  CAN1.setGeneralCallback(can1_callback);

  attachInterrupt(sensor, ISR, FALLING);
  find_home();
}

void loop()
{
  if (request_lock_low)  { digitalWrite(lock, LOW);  request_lock_low = false; }
  if (request_lock_high) { digitalWrite(lock, HIGH); request_lock_high = false; }

  // 1) handle shot request set by CAN callback
  if (shot_requested && !shot_in_progress) {
    // start the shot: move to target, start timer
    motors.set_target_pos(0, initial_encoder + one_round);
    shot_start_time = millis();
    shot_in_progress = true;
    shot_requested = false;
    // Serial.println("Shot started (moved to target).");
  }

  // 2) after returnDelay, return to initial_encoder
  if (shot_in_progress && (millis() - shot_start_time >= return_Delay)) {
    motors.set_target_pos(0, initial_encoder);
    shot_in_progress = false;
    // Serial.println("Returned to initial_encoder.");
    find_home();
  }

  // always send CAN frame
  send_rm_frame();
}
