#include <Wire.h>
#include "GimbalMotor.h"

GimbalMotor left(16, 17, 18, 33, 7);

float torque = 2.5f;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 400000);
  delay(200);

  left.setSupply(10.0f);
  left.setVoltageLimit(5.0f);

  left.init();
  left.calibrate();

  Serial.print("offset: ");
  Serial.println(left.getOffset(), 4);
}

void loop() {
  left.setTorque(torque);

  if (Serial.available()) {
    torque = Serial.parseFloat();
    Serial.print("torque: ");
    Serial.println(torque);
  }
}