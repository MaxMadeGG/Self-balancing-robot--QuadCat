#include <Wire.h>
#include <GimbalMotor.h>

GimbalMotor left(16, 17, 18, 33, 7);

float kp = 1.0f, ki = 0.0f, kd = 0.5f;
float integral = 0, lastAngle = 0;
bool  firstRun = true;

float gyroBias = 0;
float tilt = 0;
unsigned long lastT = 0;

const float BALANCE_POINT = 0.39f;
const bool  MOTOR_ENABLED = true;

void readMPU(float &accAngle, float &gyroRate) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);

  if (Wire.available() < 14) { accAngle = 0; gyroRate = 0; return; }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();

  accAngle = atan2f((float)ax, (float)az) * 57.2958f;
  gyroRate = ((float)gy / 131.0f) - gyroBias;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("BOOT");

  Wire.begin(21, 22, 400000);
  delay(200);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) Serial.println("MPU NOT FOUND");
  else Serial.println("MPU ok");
  delay(100);

  left.setSupply(10.0f);
  left.setVoltageLimit(5.0f);
  left.init();
  left.calibrate();
  if (!MOTOR_ENABLED) left.disable();

  Serial.println("Calibrating gyro - keep still...");
  float sum = 0;
  for (int i = 0; i < 500; i++) {
    float a, g;
    readMPU(a, g);
    sum += g;
    delay(2);
  }
  gyroBias = sum / 500.0f;
  Serial.print("gyro bias: ");
  Serial.println(gyroBias, 3);

  float a, g;
  readMPU(a, g);
  tilt = a;
  lastT = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastT) * 1e-6f;
  if (dt < 0.002f) return;
  lastT = now;

  float accAngle, gyroRate;
  readMPU(accAngle, gyroRate);

  tilt = 0.98f * (tilt + gyroRate * dt) + 0.02f * accAngle;

  float error = BALANCE_POINT - tilt;

  integral += error * dt;
  integral = constrain(integral, -20.0f, 20.0f);

  if (firstRun) { lastAngle = tilt; firstRun = false; }
  float derivative = -(tilt - lastAngle) / dt;
  lastAngle = tilt;

float output = constrain(kp*error - kd*gyroRate, -5.0f, 5.0f);

  if (fabsf(tilt - BALANCE_POINT) > 40.0f) {
    left.disable();
  } else if (MOTOR_ENABLED) {
    left.setTorque(output);
  }

  static unsigned long lp = 0;
  if (millis() - lp > 100) {
    lp = millis();
Serial.print("gyro: "); Serial.print(gyroRate, 2);
Serial.print("  tilt: "); Serial.print(tilt, 2);
Serial.print("  out: ");  Serial.println(output, 2);
  }
}