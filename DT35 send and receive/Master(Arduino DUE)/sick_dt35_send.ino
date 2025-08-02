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

const int DT35_PIN = A0;

const float A = -3.11941934e-07;
const float B =  1.26478998;
const float C = -158.553317;
float dist = 0.0;

float convertRawToDistance(int raw) {
  float d_cm = A * raw * raw + B * raw + C;
  float d = (d_cm > 0.0f) ? d_cm * 10.0f : 0.0f;//ensure not negative
  return d;
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
  // Serial.println(dist, 1);   
  // delay(500);
    // this function updates the data in txData
  updateDataToSend();
  // this function sends the data if one is ready to be sent
  transmitData();
}

