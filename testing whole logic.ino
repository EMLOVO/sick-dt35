#include <PS4Controller.h>
#include <esp32_can.h>
#include "vesc_set.h"
#include <ESP32Servo.h>
// #include "MecanumWheelDriver.h"
// #include "carbase_driver.h"

#define MAIN_MOTOR_ID 1

long last_time;
long last_time_us = 0;

// Rm_set motors;
// int carbase_stage = 1;
// int CarBase_RotationReduction = 6.5;  // Can Change 
// bool angle_check = false;
// MecanumWheelDriver mech_driver(0.3, 0.30, 0.08);
// Carbase carbase(4, 1.5, 1.8, 3.14);
// int direction = -1;
// int carbase_motor_rpm[4] = { 0, 0, 0, 0 };

// void send_can_frame() {
//   CAN_FRAME tx0;  // for RM motors 1-4 (carbase)
//   tx0.id = 0x200;
//   tx0.length = 8;
//   for (int i = 0; i < 8; i++) {
//     tx0.data.byte[i] = motors.can_msg0[i];
//   }
//   CAN0.sendFrame(tx0);
// }

// void can_callback(CAN_FRAME* frame) {
//   int rx_id = frame->id;
//   if (rx_id > 0x200 && rx_id < 0x209) {
//     motors.update_motor_status(rx_id - 0x201, micros(),
//       frame->data.uint8[0] << 8 | frame->data.uint8[1],
//       frame->data.uint8[2] << 8 | frame->data.uint8[3]);
//   }
// }

// void task_cantx(void* param) {
//   for (;;) {
//     send_can_frame();
//     vTaskDelay(10 / portTICK_PERIOD_MS);
//   }
// }

const float HEIGHT_PASSIVE = 1200;
const float HEIGHT_MOVING = 1000;
float x = 0;
bool passive_pole = false;
bool moving_pole = false;

// =============== PS4 Controller ===============
#define JOYSTICK_DEADZONE 15

bool current_buttons[16];
bool last_buttons[16];
int press_list[16];

int lx;  // left joystick x -128..128
int ly;  // left joystick y -128..128
int rx;  // right joystick x -128..128
int ry;  // right joystick y -128..128

void notify() {

  lx = (PS4.LStickX() * direction * -1);
  ly = (PS4.LStickY() * direction);
  rx = (PS4.RStickX() * -1);
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

  // Buttons R2 and L2
  // if (current_buttons[11] && current_buttons[10]) {
  //   carbase_stage = 0;
  // } else {
  //   carbase_stage = 1;
  // }
  // carbase.max_v = stage_max_v[carbase_stage];
  // carbase.max_w = stage_max_w[carbase_stage];

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
  carbase.update_from_controller(0, 0, 0);
}

// void task_update_carbase(void* param) {
//   for (;;) {
//     if(!angle_check){
//     carbase.update_from_controller(lx, ly, rx / CarBase_RotationReduction);  // Carbase update
//     }else{
//       if(theta-init_theta >= 0.005 * PI){
//         carbase.update_from_controller(0,0,-128 / CarBase_RotationReduction);  // Carbase update
//         Serial.println(theta);
//       }else if(init_theta - theta >= 0.005 * PI){
//         carbase.update_from_controller(0,0,128 / CarBase_RotationReduction);  // Carbase update
//       }else{
//         angle_check = !angle_check;  // Carbase update
//       }
//     }
//     vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

//servo
Servo myServo;
const int SERVO_PIN = 25;
int servo_angle = 0;
int last_servo_angle = -1;
int servoangle1[] = {0,140};
bool servo_toggle = false;

//5065
Vesc_set main_motor;
bool shooted = false;
bool reset = false;
double shooter_rpm = 12000;

void send_can0_frame(Vesc_set &motor) {
  CAN_FRAME tx_msg;
  tx_msg.id = motor.MOTOR_ID;
  tx_msg.extended = true;
  tx_msg.length = motor.len;
  memcpy(tx_msg.data.byte, motor.can_msg, 8);
  CAN0.sendFrame(tx_msg);
}

void task_cantx1(void* param) {
  for (;;) {
    send_can_frame2(main_motor);
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void shoot_shooter(){
  // Serial.println(shooter_rpm);
  //main_motor.comm_can_set_rpm(MAIN_MOTOR_ID, shooter_rpm);
  //send_can0_frame(main_motor);
  
  if(shooted){
  // 1) 射擊：設定 RPM
  main_motor.comm_can_set_rpm(MAIN_MOTOR_ID, shooter_rpm);
  send_can0_frame(main_motor);
  delay(500);
// 2) 停馬：停止輸出
  main_motor.comm_can_set_rpm(MAIN_MOTOR_ID, 0.0);
  send_can0_frame(main_motor);
// 3) 回原點
  main_motor.comm_can_set_rpm(MAIN_MOTOR_ID, -3000);
  CAN0.sendFrame(main_motor);
  delay(10);
// 4) 到位後，停並清除旗標
  main_motor.comm_can_set_rpm(MAIN_MOTOR_ID, 0.0);
  CAN0.sendFrame(MAIN_MOTOR_ID);
  shooted = false;
  }
  delay(20);
}

// void auto_aim_shoot(float x, float y){

// }

const int dt35Pin = 34;  // 接 DT35 analog 輸出的 GPIO pin
const float VREF = 3.3;    // ESP32 的 ADC 參考電壓
const uint16_t ADC_RESOLUTION = 4095; // ESP32 是 12-bit ADC



// Convert raw sensor data to mm (we will update with your real calibration later)
float convertRawToDistance(int raw_value) {
  // Placeholder linear mapping: real formula needs calibration
  return map(raw_value, 5, 648, 20, 6500);  // map raw 5-648 to 20-6500 mm
}

// Calculate angle to shoot
float calculateShootAngle(float x_mm, float y_mm) {
  float angle_rad = atan2(y_mm, x_mm);
  float angle_deg = angle_rad * 180.0 / PI;
  return angle_deg;
}

// Map angle in degrees to servo signal (you need to calibrate this too)
int angleToServo(int angle_deg) {
  return constrain(map(angle_deg, 0, 90, 0, 180), 0, 180); // Servo 0~180
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.println("Hello, ESP32!");
  CAN0.begin(1000000);
  CAN0.watchFor();
  CAN0.setGeneralCallback(can0_callback);

  // PS4 controller
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);
  PS4.begin("0a:0a:0a:0a:0a:0a");  

  myServo.attach(SERVO_PIN);
  myServo.write(0);

// xTaskCreatePinnedToCore(
//     task_cantx,  // Function that should be called
//     "cantx",     // Name of the task (for debugging)
//     4096,        // Stack size (bytes)
//     NULL,        // Parameter to pass
//     1,           // Task priority
//     NULL,        // Task handle
//     1            // Core you want to run the task on (0 or 1)
//   );

  xTaskCreatePinnedToCore(
    task_cantx1,  // Function that should be called
    "cantx1",     // Name of the task (for debugging)
    4096,        // Stack size (bytes)
    NULL,        // Parameter to pass
    1,           // Task priority
    NULL,        // Task handle
    0            // Core you want to run the task on (0 or 1)
  );

  // xTaskCreatePinnedToCore(
  //   task_update_carbase,  // Function that should be called
  //   "cantx2",     // Name of the task (for debugging)
  //   4096,        // Stack size (bytes)
  //   NULL,        // Parameter to pass
  //   1,           // Task priority
  //   NULL,        // Task handle
  //   0            // Core you want to run the task on (0 or 1)
  // );

}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(10); // this speeds up the simulation
  // carbase.tick(micros());
  // mech_driver.getMovement(carbase_motor_rpm, carbase.current_v[0], carbase.current_v[1], carbase.current_w);
  int raw = analogRead(dt35Pin);
  float x = convertRawToDistance(raw);//will change later, the table not yet finish
  
  if(PS4.Right() == true){
      passive_mode = true;
      moving_mode = false;
  }
  if(PS4.Left() == true){
      passive_mode = false;
      moving_mode = true;
  }  
  if (passive_pole) {
    float angle_deg = calculateShootAngle(x, HEIGHT_PASSIVE);
    int servo_pos = angleToServo(angle_deg);
    myServo.write(servo_pos);

    delay(300);  // wait for servo to settle
    
    shoot_shooter();

    passive_mode = false;
  }  
  
  if (moving_pole) {
    float angle_deg = calculateShootAngle(x, HEIGHT_MOVING);
    int servo_pos = angleToServo(angle_deg);
    myServo.write(servo_pos);

    delay(300);  // wait for servo to settle
    
    shoot_shooter();

    moving_mode = false;
  }

  delay(50);
  
  
  // if(PS4.Cross()==true){
  //   shooted = true;
  //   shoot_shooter();//auto_aim_shoot
  // }
  // if(shooted == true){
    
  
  //   delay(500)    // 保持射擊時間
  // }

}
