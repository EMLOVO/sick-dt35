#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>

#define lock 13
#define sensor 32
#define POSITION_TOLERANCE 400L 

struct I2cRxStruct {
          float  Distance;
          byte padding[10];
  //------

};

I2cRxStruct rxData;

bool newRxData = false;


        // I2C control stuff
#include <Wire.h>

const byte thisAddress = 0x09; // these need to be swapped for the other Arduino
//Default SDA port = gpio21
//Default SCL port = gpio22
//=================================

const float DIST_MIN = 90.0f;  // 下限（示範值）
const float DIST_MAX = 110.0f;  // 上限（示範值）
bool armedBySquare = false; 

// clearer lock macros
#define LOCKED LOW
#define UNLOCKED HIGH

bool lockState = UNLOCKED;

const unsigned long return_Delay = 2000UL;
const unsigned long homed_Delay = 2000UL;
Rm_set motors;

volatile int homing_stage = -1;
volatile int initial_stage = -1;
volatile long initial_encoder = 0; // latched in CAN callback

volatile bool shot_requested = false;
bool shot_in_progress = false;
// unsigned long shot_start_time = 0;
unsigned long home_start_time = 0;

int top = -400000; // keep your sign
int last_time = 0;
#define JOYSTICK_DEADZONE 12
int lx, ly, rx, ry;

bool current_buttons[16];
bool last_buttons[16];
int press_list[16];

// new flag: true when we intentionally want to run sensor-based homing as a return
volatile bool returning_via_sensor = false;

enum MachineState {
  STATE_HOMING,
  STATE_HOMED_LOCKED,
  STATE_MOVING_TO_SHOT,
  STATE_AT_SHOT_LOCKED,
  STATE_UNLOCKED,
  STATE_RETURNING_HOME,        // previous absolute-return
  STATE_RETURNING_VIA_SENSOR   // sensor-based return/homing
};
volatile MachineState state = STATE_HOMING;

long shot_target_pos = 0;

// new: timestamp when entering STATE_UNLOCKED
unsigned long unlock_time = 0;
const unsigned long UNLOCK_AUTO_TIMEOUT = 2000UL; // 2 seconds

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
    press_list[i] = (int)current_buttons[i] - (int)last_buttons[i]; // -1 released, +1 pressed
  }

  if (press_list[5] && state == STATE_AT_SHOT_LOCKED) { // TRIANGLE rising edge
    Serial.println(state);
    digitalWrite(lock, UNLOCKED);
    lockState = UNLOCKED;
    state = STATE_UNLOCKED;
    unlock_time = millis(); // start the 2-second timer
    Serial.println("TRIANGLE pressed: UNLOCK (lock HIGH). Starting 2s auto-return timer.");
    motors.set_target_rpm(4, 0); // optional: release PID so it won't fight manual motion
  }

  if (press_list[6] && state == STATE_AT_SHOT_LOCKED) {
      armedBySquare = true;
      Serial.println("Square pressed: armed waiting for distance...");
  }
}
  
  for (int i = 0; i < 16; i++) last_buttons[i] = current_buttons[i];

    // --- Triangle pressed -> unlock (HIGH)  (press_list[5] per your mapping)

}

void onConnect() {
  PS4.setLed(255, 0, 0);
  PS4.sendToController();
  Serial.println("PS4 connected");
}
void onDisconnect() {
  Serial.println("PS4 disconnected");
}

void IRAM_ATTR ISR()
{
  // sensor falling interrupt: marks that sensor was hit
  if (homing_stage > 0) {
    homing_stage = 0;
    initial_stage = 0; // will be processed inside CAN callback to latch encoder
  }
}

void find_home()
{
  homing_stage = 1;
  state = STATE_HOMING;
  Serial.println("find_home() called -> homing_stage set to 1");
}

void task_cantx(void* param) {
  for (;;) {
    // keep CAN frames flowing
    send_rm_frame();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void send_rm_frame()
{
  //if rm id = 1-4 -> tx_msg0 (0x200)
  //if rm id = 5-8 -> tx_msg1 (0x1FF)

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
  //delayMicroseconds(100);

}

void can0_callback(CAN_FRAME *frame)
{
  int rx_id = frame->id;
  if (rx_id > 0x200 && rx_id < 0x209) {
    motors.update_motor_status(rx_id - 0x201, micros(), frame->data.uint8[0] << 8 | frame->data.uint8[1], frame->data.uint8[2] << 8 | frame->data.uint8[3]);
  } 

  // while homing_stage==1, drive toward sensor (same behaviour as before)
  if (homing_stage == 1)
  {
    motors.set_target_rpm(4, 1000); // drive toward sensor; tune rpm if needed
  }

  // When initial_stage==0 (ISR has signalled sensor hit), latch encoder and stop
  if (initial_stage == 0)
  {
    motors.set_target_rpm(4, 0);             // stop
    initial_encoder = motors.gearbox_pos[4]; // latch home
    initial_stage = 1;
    shot_requested = true; // preserve original behaviour (auto shot if you want)
    home_start_time = millis();

    // Now lock (LOW) after homing and indicate state
    digitalWrite(lock, LOCKED);
    lockState = LOCKED;

    // if we were returning via sensor, mark it done
    if (returning_via_sensor) {
      returning_via_sensor = false;
      state = STATE_HOMED_LOCKED;
      Serial.printf("RETURN VIA SENSOR: latched initial_encoder=%ld -> LOCKED\n", initial_encoder);
    } else {
      // normal initial homing
      state = STATE_HOMED_LOCKED;
      Serial.printf("HOMED (normal): initial_encoder=%ld -> LOCKED\n", initial_encoder);
    }
  }
}

// this function is called by the Wire library when a message is received
void receiveEvent(int numBytesReceived) {

    if (newRxData == false) {
            // copy the data to rxData
        Wire.readBytes( (byte*) &rxData, numBytesReceived);
        newRxData = true;
    }
    else {
            // dump the data
        while(Wire.available() > 0) {
            byte c = Wire.read();
        }
    }
}

void setup()
{
  pinMode(sensor, INPUT);
  pinMode(lock, OUTPUT);

  // start unlocked until homing completes; homing callback will lock
  digitalWrite(lock, UNLOCKED);
  lockState = UNLOCKED;

  attachInterrupt(sensor, ISR, FALLING);

      // set up I2C
  Wire.begin(thisAddress); // join i2c bus
  Wire.onReceive(receiveEvent); // register event
  
  Serial.begin(115200);
  PS4.begin("a0:a0:a0:a0:a0:a0");
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);

  for (int i = 0; i < motors.MOTOR_NUM; i++) {
    motors.set_pid_rpm(i, 6, 0.01, 0);
    motors.reset_gearbox_pos(i);
  }

  CAN0.begin(1000000);
  CAN0.watchFor();
  CAN0.setGeneralCallback(can0_callback);

  xTaskCreatePinnedToCore(
    task_cantx,  // Function that should be called
    "cantx",     // Name of the task (for debugging)
    2048,        // Stack size (bytes)
    NULL,        // Parameter to pass
    1,           // Task priority
    NULL,        // Task handle
    1            // Core you want to run the task on (0 or 1)
  );

  // initial homing on startup
  find_home();
}

void start_return_via_sensor() {
  // helper to start returning via sensor (prevent duplicates)
  if (state == STATE_RETURNING_VIA_SENSOR || returning_via_sensor) return;
  Serial.println("Starting return via sensor (triggered).");
  // ensure unlocked
  digitalWrite(lock, UNLOCKED);
  lockState = UNLOCKED;
  delay(10);
  returning_via_sensor = true;
  state = STATE_RETURNING_VIA_SENSOR;
  find_home(); // sets homing_stage=1; can0_callback will drive toward sensor when frame arrives
}

void loop()
{
  if (micros() - last_time > 20000) {
    last_time = micros();
  // debug prints -- once every 250 ms
  // static unsigned long last_dbg = 0;
  // if (millis() - last_dbg >= 250) {
  //   long cur = motors.gearbox_pos[4];
  //   Serial.printf("STATE=%d CUR=%ld INIT=%ld lock=%d returning_via_sensor=%d\n",
  //                 (int)state, cur, initial_encoder, digitalRead(lock), (int)returning_via_sensor);
  //   last_dbg = millis();
  // }
    
  // Handle received I2C distance data (copy to local var safely)
  // -----------------------
  static float distance_local = 0.0f;
  if (newRxData) {
    // copy into local (non-volatile) var for processing
    distance_local = rxData.Distance;
    newRxData = false;
    // optional debug
    // Serial.printf("I2C DIST RECEIVED: %.3f\n", distance_local);
  }
    
  // --- automatic move to shot after homed (unchanged)
  if (shot_requested && !shot_in_progress && (millis() - home_start_time >= homed_Delay))
  {
    shot_target_pos = initial_encoder + top;
    motors.set_target_pos(4, shot_target_pos);
    // shot_start_time = millis();
    shot_in_progress = true;
    shot_requested = false;
    state = STATE_MOVING_TO_SHOT;
    Serial.printf("AUTO: Moving to shot target pos=%ld\n", shot_target_pos);
  }

  // monitor moving-to-shot arrival (inline abs diff)
  if (state == STATE_MOVING_TO_SHOT) {
    long cur = motors.gearbox_pos[4];
    long diff = (cur > shot_target_pos) ? (cur - shot_target_pos) : (shot_target_pos - cur);
    if (diff <= POSITION_TOLERANCE) {
      shot_in_progress = false;
      state = STATE_AT_SHOT_LOCKED;
      Serial.println("Arrived at shot target (locked state).");
  }
}
    
// --- If armed by Square, wait until distance is in range ---
if (armedBySquare && state == STATE_AT_SHOT_LOCKED) {
    if (distance_local> DIST_MIN && distance_local < DIST_MAX) {
        digitalWrite(lock, UNLOCKED);
        lockState = UNLOCKED;
        state = STATE_UNLOCKED;
        unlock_time = millis();
        armedBySquare = false; // reset
        Serial.println("Distance in range -> Auto UNLOCK (armed by Square)");
        motors.set_target_rpm(4, 0);
    }
}

  // --- AUTO: if unlocked for >= 2s, start return via sensor
  if (state == STATE_UNLOCKED) {
    if ((millis() - unlock_time >= UNLOCK_AUTO_TIMEOUT) && !returning_via_sensor) {
      Serial.println("STATE_UNLOCKED timeout reached -> auto starting return via sensor.");
      start_return_via_sensor();
    }
  }

  }
}
