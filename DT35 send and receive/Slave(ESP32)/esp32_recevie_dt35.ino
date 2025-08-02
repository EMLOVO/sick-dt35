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

void setup() {
    Serial.begin(115200);
    // set up I2C
    Wire.begin(thisAddress); // join i2c bus
    Wire.onReceive(receiveEvent); // register event
}

//============

void loop() {

        // this bit checks if a message has been received
    if (newRxData == true) {
        showNewData();
        newRxData = false;
    }
}


//=============

void showNewData() {

    Serial.print("Distance: ");
    Serial.print(rxData.Distance);
    Serial.println(' ');

}

//============

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