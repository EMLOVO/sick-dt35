#include "rm_set.h"
#include <esp32_can.h>

#define clamp 13
#define sensor 32
Rm_set motors;
int initial_pos;

const int down = 400000;

int homing_stage_y = -1;
int initial_stage_y = -1;
int y_initial_encoder = 0;

void IRAM_ATTR ISR_Y() {
  if (homing_stage_y > 0) {
    homing_stage_y = 0;
    initial_stage_y = 0;
  }
}

void find_home() {
  homing_stage_y = 1;
  digitalWrite(clamp,HIGH);

}

void send_rm_frame() {
  CAN_FRAME tx_msg0;
  tx_msg0.id = 0x200;
  tx_msg0.length = 8;
  for (int i = 0; i < 8; i++) {
    tx_msg0.data.byte[i] = motors.can_msg0[i];
  }
  CAN1.sendFrame(tx_msg0);
}

void can1_callback(CAN_FRAME *frame)
{
    // rm motor
    int rx_id = frame->id;
    motors.update_motor_status(rx_id - 0x201, micros(), frame->data.uint8[0] << 8 | frame->data.uint8[1], frame->data.uint8[2] << 8 | frame->data.uint8[3]);

  if (homing_stage_y == 1) {
      motors.set_target_rpm(0, 1000);
    } 

  if (initial_stage_y == 0) {
    motors.set_target_rpm(0, 0);
    y_initial_encoder = motors.gearbox_pos[0];
    //Serial.printf("read!");
    // digitalWrite(clamp, HIGH);
    initial_stage_y = 1;
  }


  
}

void setup()
{
  pinMode(sensor,INPUT);
  pinMode(clamp,OUTPUT);
  digitalWrite(clamp, LOW);
  Serial.begin(115200);
  for (int i = 0; i < motors.MOTOR_NUM; i++) {
    motors.set_pid_rpm(i, 6, 0.01, 0); //set pid for each rm aqW22
    motors.reset_gearbox_pos(i); //reset the encoder position to 0
  }
  digitalWrite(clamp, LOW);
  CAN1.begin(1000000);   
  CAN1.watchFor();
  CAN1.setGeneralCallback(can1_callback);

  Serial.println(motors.gearbox_pos[0]);
  attachInterrupt(sensor, ISR_Y, FALLING);

  find_home();
}

void loop()
{

    // if (Serial.available()) {
    //   String i = Serial.readStringUntil('\r\n');
    //   int input = i.toInt();

    //     for (int i = 0; i < 1; i++)//motors.MOTOR_NUM
    //     {
    //       motors.set_target_pos(0, initial_pos + input); // 較 position , 157318 = 1 個圈 
    //     }
    // }


    // if(digitalRead(sensor)==1){

    //   Serial.println("1");
    // }else
    // {
    //   Serial.println("0");
    // }

    // Serial.println(motors.gearbox_pos[0]);
    //Serial.println(digitalRead(sensor));
    send_rm_frame(); // 記得send番個rm frame
}