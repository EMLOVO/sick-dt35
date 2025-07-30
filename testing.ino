const int analogPin = A0;

// 用你实测的 near/far raw
const int RAW_MIN    = 119;   // 对应  2 cm
const int RAW_MAX    = 652;   // 对应 650 cm
const float DIST_MIN =   2.0; // cm
const float DIST_MAX = 650.0; // cm

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(analogPin);
  raw = constrain(raw, RAW_MIN, RAW_MAX);

  float distance = (raw - RAW_MIN)  
                 * (DIST_MAX - DIST_MIN)  
                 / float(RAW_MAX - RAW_MIN)  
                 + DIST_MIN;

  Serial.print("Raw = ");
  Serial.print(raw);
  Serial.print(" | Distance = ");
  Serial.print(distance, 1);
  Serial.println(" cm");

  delay(200);
}
