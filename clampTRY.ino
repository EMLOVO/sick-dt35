#include "rm_set.h"
#include <esp32_can.h>
#include <PS4Controller.h>

#define clamp 13

Rm_set motors;
long initial_pos = 0;
bool reload_in_progress = false;
int reload_step = 0;
unsigned long step_timer = 0;

// target position offset for loading
const long LOAD_OFFSET = 10000;  
long target_pos = 0;

void setup() {
  Serial.begin(115200);

  pinMode(clamp, OUTPUT);
  digitalWrite(clamp, LOW); // open initially

  motors.begin();
  initial_pos = motors.gearbox_pos[0];
  target_pos = initial_pos + LOAD_OFFSET;

  PS4.begin("01:02:03:04:05:06");
  Serial.println("Setup done, waiting for PS4...");
}

void loop() {
  motors.update();

  // X button = clamp close manually
  if (PS4.Cross()) {
    digitalWrite(clamp, HIGH);
    Serial.println("Clamp closed (manual)");
  }

  // Square button = start reload sequence
  if (PS4.Square() && !reload_in_progress) {
    reload_in_progress = true;
    reload_step = 0;
    step_timer = millis();
    Serial.println("Reload sequence started");
  }

  if (reload_in_progress) {
    unsigned long now = millis();

    switch (reload_step) {
      case 0: // move motor to target pos
        motors.set_pos(0, target_pos);
        if (abs(motors.gearbox_pos[0] - target_pos) < 500) {
          reload_step++;
          step_timer = now;
          Serial.println("Reached load pos");
        }
        break;

      case 1: // open clamp
        digitalWrite(clamp, LOW);
        if (now - step_timer > 300) { // wait 300ms
          reload_step++;
          step_timer = now;
          Serial.println("Clamp open 1");
        }
        break;

      case 2: // close clamp
        digitalWrite(clamp, HIGH);
        if (now - step_timer > 300) {
          reload_step++;
          step_timer = now;
          Serial.println("Clamp close 1");
        }
        break;

      case 3: // open clamp again
        digitalWrite(clamp, LOW);
        if (now - step_timer > 300) {
          reload_step++;
          step_timer = now;
          Serial.println("Clamp open 2");
        }
        break;

      case 4: // close clamp
        digitalWrite(clamp, HIGH);
        if (now - step_timer > 300) {
          reload_step++;
          step_timer = now;
          Serial.println("Clamp close 2");
        }
        break;

      case 5: // move back to initial pos
        motors.set_pos(0, initial_pos);
        if (abs(motors.gearbox_pos[0] - initial_pos) < 500) {
          reload_step++;
          Serial.println("Back to initial pos");
        }
        break;

      case 6: // done
        reload_in_progress = false;
        Serial.println("Reload done");
        break;
    }
  }

  motors.send_can_frame();
}
