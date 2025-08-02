// const int analogPin = A0;

// // 用你实测的 near/far raw
// const int RAW_MIN    = 119;   // 对应  2 cm
// const int RAW_MAX    = 652;   // 对应 650 cm
// const float DIST_MIN =   2.0; // cm
// const float DIST_MAX = 650.0; // cm

// void setup() {
//   Serial.begin(115200);
// }

// void loop() {
//   int raw = analogRead(analogPin);
//   raw = constrain(raw, RAW_MIN, RAW_MAX);

//   float distance = (raw - RAW_MIN)  
//                  * (DIST_MAX - DIST_MIN)  
//                  / float(RAW_MAX - RAW_MIN)  
//                  + DIST_MIN;

//   Serial.print("Raw = ");
//   Serial.print(raw);
//   Serial.print(" | Distance = ");
//   Serial.print(distance, 1);
//   Serial.println(" cm");

//   delay(200);
// }


// const int DT35_PIN = A0;    
  

// void setup() {
//   Serial.begin(115200);
// }

// void loop() {
//   int raw = analogRead(DT35_PIN);
//   Serial.println(raw);
//   delay(500);
// }
struct I2cTxStruct {
  float  Distance;
  byte padding[10];
};

I2cTxStruct txData;

bool newTxData = false;

#include <Wire.h>

#define i2C_pin_SDA_1 21
#define i2C_pin_SCL_1 22

const byte thisAddress = 0x08; // these need to be swapped for the other Arduino
const byte otherAddress1 = 0x09; // address of Slave 1

unsigned long prevUpdateTime = 0;
unsigned long updateInterval = 150;

// ———— 验证 raw→distance 拟合准确性 ————
const int DT35_PIN = A0;     // Q2 分压后接 A0

// 拟合系数（从你的数据用二次回归算出）
const float A = -3.11941934e-07;
const float B =  1.26478998;
const float C = -158.553317;
float dist = 0.0;

// 把 raw(ADC) 转成距离 (cm)
float convertRawToDistance(int raw) {
  float d = A * raw * raw + B * raw + C;
  return (d > 0) ? d : 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // join i2c bus
}

void updateDataToSend() {

  if (millis() - prevUpdateTime >= updateInterval) {
    prevUpdateTime = millis();
    if (newTxData == false) { // ensure previous message has been sent
      txData.Distance = dist;
      newTxData = true;
    }
  }
}

void transmitData() {

  if (newTxData == true) {
    Wire.beginTransmission(otherAddress1);
    Wire.write((byte*) &txData, sizeof(txData));
    Wire.endTransmission();    // this is what actually sends the data

    // for demo show the data that as been sent
    Serial.print("Sent ");
    Serial.print(txData.Distance);
    Serial.println(' ');
    newTxData = false;
  }
}

void loop() {
  int raw = analogRead(DT35_PIN);
  dist = convertRawToDistance(raw);
  // Serial.print(raw);
  // Serial.print('\t');
  // Serial.println(dist, 1);    // 保留 1 位小数
  // delay(500);
    // this function updates the data in txData
  updateDataToSend();
  // this function sends the data if one is ready to be sent
  transmitData();
}

