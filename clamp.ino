#include "rm_set.h" 
#include <esp32_can.h>
#include <PS4Controller.h>

#define clamp 14
#define JOYSTICK_DEADZONE 12

Rm_set motors;
long int_pos = 0;               // latched initial/home position (long matches gearbox_pos)
long current_pos = 0;           // live pos if needed
int lx, ly, rx, ry;
int targetPos = 0;
bool up = false;

bool current_buttons[16];
bool last_buttons[16];
int press_list[16];

volatile bool clampClosed = false;

int pos_A = 120000;
int pos_B = 90000;

// latch flag: true after initial position recorded once from CAN feedback
volatile bool homeLatched = false;

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
  current_buttons[6] = PS4.Square();   // start load sequence
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
    press_list[i] = current_buttons[i] - last_buttons[i]; // -1 released, +1 pressed
  }
  for (int i = 0; i < 16; i++) last_buttons[i] = current_buttons[i];

  // toggle clamp on rising edge only
  if (press_list[4] == 1) {
    clampClosed = !clampClosed;
    Serial.print("Clamp toggled. Now closed? ");
    Serial.println(clampClosed ? "YES" : "NO");
  }

  // Start load sequence: D-pad Up moves up relative to latched int_pos
  if (press_list[1] == 1 && clampClosed) {
    Serial.println("up!");
    motors.set_target_pos(5, int_pos + pos_A);
    up = true;
  }

  // D-pad Down moves back to latched int_pos (only if we previously went up)
  if (press_list[3] == 1 && up) {
    Serial.println("down!");
    motors.set_target_pos(5, int_pos + pos_B);
    up = false;
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

void loop()
{
  // reflect clamp output
  digitalWrite(clamp, clampClosed ? LOW : HIGH);

  // debug (less spammy)
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg >= 500) {
    Serial.print("live[5]=");
    Serial.print(current_pos);
    Serial.print("  latched_home=");
    Serial.print(int_pos);
    Serial.print("  clampClosed=");
    Serial.print(clampClosed);                 
    Serial.print("  up=");
    Serial.println(up);
    lastDbg = millis();
  }

  send_rm_frame(); // keep CAN frames flowing
}
