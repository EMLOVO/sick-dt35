#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>

#define clamp 14
#define JOYSTICK_DEADZONE 12

Rm_set motors;
long int_pos = 0;               // latched home/reference (set once)
long current_pos = 0;           // live feedback
int lx, ly, rx, ry;
bool up = false;

bool current_buttons[16];
bool last_buttons[16];
int press_list[16];

volatile bool clampClosed = false;
volatile bool homeLatched = false;

int pos_A = 130000;
int pos_B = 90000;

void notify()
{
  // (keep your existing joystick/button reads here, unchanged)
  lx = PS4.LStickX(); ly = PS4.LStickY(); rx = PS4.RStickX(); ry = PS4.RStickY();
  if (abs(lx) < JOYSTICK_DEADZONE) lx = 0;
  if (abs(ly) < JOYSTICK_DEADZONE) ly = 0;
  if (abs(rx) < JOYSTICK_DEADZONE) rx = 0;
  if (abs(ry) < JOYSTICK_DEADZONE) ry = 0;

  // fill current_buttons[] as you already do...
  current_buttons[0] = PS4.Right(); current_buttons[1] = PS4.Up();
  current_buttons[2] = PS4.Left();  current_buttons[3] = PS4.Down();
  current_buttons[4] = PS4.Circle(); current_buttons[5] = PS4.Triangle();
  current_buttons[6] = PS4.Square(); current_buttons[7] = PS4.Cross();
  current_buttons[8] = PS4.L1(); current_buttons[9] = PS4.R1();
  current_buttons[10] = PS4.L2(); current_buttons[11] = PS4.R2();
  current_buttons[12] = PS4.Options(); current_buttons[13] = PS4.Share();
  current_buttons[14] = PS4.PSButton(); current_buttons[15] = PS4.Touchpad();

  for (int i = 0; i < 16; i++) press_list[i] = (int)current_buttons[i] - (int)last_buttons[i];
  for (int i = 0; i < 16; i++) last_buttons[i] = current_buttons[i];

  // clamp toggle on rising edge
  if (press_list[4] == 1) {
    clampClosed = !clampClosed;
    Serial.print("Clamp toggled manually. Now closed? ");
    Serial.println(clampClosed ? "YES" : "NO");
  }

  // UP (same as before) - send frame immediately after set_target_pos
  if (press_list[1] == 1) {
    if (!homeLatched) {
      Serial.println("IGNORED UP: home not latched yet (wait for CAN feedback).");
    } else if (!clampClosed) {
      Serial.println("IGNORED UP: clamp is open; close clamp first.");
    } else {
      long target_A = int_pos + pos_A;
      Serial.print("UP pressed -> target=");
      Serial.print(target_A);
      Serial.print(" (int_pos=");
      Serial.print(int_pos);
      Serial.print(" current_pos=");
      Serial.print(current_pos);
      Serial.println(")");
      motors.set_target_rpm(5, 0);         // sometimes help "arm" torque
      motors.set_target_pos(5, target_A);
      send_rm_frame();                     // force immediate send
      Serial.println("UP: sent CAN frames.");
      up = true;
    }
  }

  // DOWN: allow if either up==true OR the motor is already above a threshold
  if (press_list[3] == 1) {
    // debug print showing states
    Serial.print("DOWN press detected: up=");
    Serial.print(up);
    Serial.print(" homeLatched=");
    Serial.print(homeLatched);
    Serial.print(" current_pos=");
    Serial.println(current_pos);

    if (!homeLatched) {
      Serial.println("IGNORED DOWN: home not latched yet.");
    } else {
      // allow down if previously went up OR if encoder currently higher than home + half pos_B
      bool consider_up = up || (current_pos > (int_pos + pos_B/2));
      if (!consider_up) {
        Serial.println("IGNORED DOWN: arm not up (up flag false and position not high enough).");
      } else {
        long target_B = int_pos + pos_B;
        Serial.print("DOWN pressed -> target=");
        Serial.print(target_B);
        Serial.print(" (int_pos=");
        Serial.print(int_pos);
        Serial.print(" current_pos=");
        Serial.print(current_pos);
        Serial.println(")");
        motors.set_target_rpm(5, 0);
        motors.set_target_pos(5, target_B);
        send_rm_frame();                     // force immediate send
        Serial.println("DOWN: sent CAN frames.");
        up = false;                          // consume the up state
      }
    }
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
    int rx_id = frame->id;
    motors.update_motor_status(rx_id - 0x201,
                               micros(),
                               frame->data.uint8[0] << 8 | frame->data.uint8[1],
                               frame->data.uint8[2] << 8 | frame->data.uint8[3]);

    // update live position
    current_pos = motors.gearbox_pos[5];

    // latch initial/home/reference position only once (first valid CAN feedback)
    if (!homeLatched && current_pos != 0) { // require non-zero to avoid spurious 0
      int_pos = current_pos;
      homeLatched = true;
      Serial.print("Home latched once from CAN: ");
      Serial.println(int_pos);
    }
}

void task_cantx(void* param) {
  for (;;) {
    // keep CAN frames flowing
    send_rm_frame();
    vTaskDelay(1 / portTICK_PERIOD_MS);
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
        motors.set_pid_rpm(i, 3, 0.01, 0); // increase RPM param for stronger motion if needed
        motors.reset_gearbox_pos(i);       // reset each motor's encoder position
    }

    Serial.println("Setup done. Waiting for first CAN feedback to latch home position...");

      xTaskCreatePinnedToCore(
    task_cantx,  // Function that should be called
    "cantx",     // Name of the task (for debugging)
    2048,        // Stack size (bytes)
    NULL,        // Parameter to pass
    1,           // Task priority
    NULL,        // Task handle
    1            // Core you want to run the task on (0 or 1)
  );
  
}

void loop()
{
  // reflect clamp output
  digitalWrite(clamp, clampClosed ? LOW : HIGH);


  // send_rm_frame(); // keep CAN frames flowing
}
