#include <Wire.h>
#include <GimbalMotor.h>

// ---- pin map as measured on board v2, not as the README had it ----
// M1: IN1 16, IN2 17, IN3 18, EN 5     (IN3/EN were swapped in the old map)
// M2: IN1 25, IN2 26, IN3 14, EN 27
// Bus 0: SDA 21, SCL 19  — MPU 0x68, encoder A 0x36, lasers
// Bus 1: SDA 22, SCL 23  — encoder B 0x36 only
GimbalMotor left(16, 17, 18, 5, 7, Wire);

// ---- control ----
const uint32_t PERIOD_US = 500;            // 2 kHz, fixed. A measured dt gave
const float    DT        = PERIOD_US * 1e-6f;   // +-19 rad/s velocity spikes.

float kp = 1.0f;
float kd = 0.5f;

const float BALANCE_POINT = 0.39f;         // degrees
const float FALL_LIMIT    = 40.0f;         // degrees from balance point
const float MAX_VOLTS     = 4.0f;
const bool  MOTOR_ENABLED = true;
const uint32_t RUN_MS     = 40000;         // gimbal motors cook at standstill

// ---- state ----
float gyroBias = 0;
float tilt     = 0;
uint32_t runStart = 0;
bool  fallen = false;

// Returns false on I2C failure. Never hand back zeros — a dead bus must not
// look like a level robot.
bool readMPU(float &accAngle, float &gyroRate) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)0x68, (uint8_t)14) != 14) return false;
  if (Wire.available() < 14) return false;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                 // ay unused: Y is the balance axis,
  int16_t az = (Wire.read() << 8) | Wire.read();   // so tilt is about Y
  Wire.read(); Wire.read();                 // temperature
  Wire.read(); Wire.read();                 // gx
  int16_t gy = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                 // gz

  accAngle = atan2f((float)ax, (float)az) * 57.2958f;
  gyroRate = ((float)gy / 131.0f) - gyroBias;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\nbalance");

  Wire.begin(21, 19, 400000);               // SCL is 19, not 22
  Wire.setTimeOut(10);
  delay(200);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);                         // wake
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU NOT FOUND — stopping");
    while (1) delay(1000);
  }
  Serial.println("MPU ok");
  delay(100);

  left.setSupply(10.0f);
  left.setVoltageLimit(MAX_VOLTS);
  left.init();                              // EN low, then PWM

  Serial.print("encoder AGC: ");
  Serial.println(left.encoderAGC());        // ~120 good, 255 too far, 0 too close

  if (!left.calibrate()) {                  // the check that caught M1
    Serial.printf("CALIBRATION FAILED  mech revs %.4f  implied poles %.2f\n",
                  left.getMechRevs(), left.getImplied());
    Serial.println("rotor not turning — check phases with a meter");
    left.disable();
    while (1) delay(1000);
  }
  Serial.printf("calibrated  offset %.4f  encDir %+d  implied poles %.2f\n",
                left.getOffset(), left.getEncDir(), left.getImplied());

  Serial.println("gyro bias — keep still");
  float sum = 0;
  int n = 0;
  for (int i = 0; i < 500; i++) {
    float a, g;
    if (readMPU(a, g)) { sum += g; n++; }
    delay(2);
  }
  if (n < 400) { Serial.println("MPU unreliable — stopping"); while (1) delay(1000); }
  gyroBias = sum / n;
  Serial.printf("gyro bias: %.3f\n", gyroBias);

  float a, g;
  if (!readMPU(a, g)) { Serial.println("MPU read failed"); while (1) delay(1000); }
  tilt = a;

  if (MOTOR_ENABLED) left.enable();         // calibrate() leaves it disabled
  runStart = millis();
  Serial.println("running");
}

void loop() {
  if (millis() - runStart > RUN_MS || fallen) {
    left.disable();
    delay(1000);
    return;
  }

  // Fixed-rate tick. DT is constant, so velocity and the filter are stable.
  static uint32_t tNext = 0;
  uint32_t now = micros();
  if (tNext == 0) tNext = now;
  if ((int32_t)(now - tNext) < 0) return;
  tNext += PERIOD_US;
  if ((int32_t)(now - tNext) > (int32_t)PERIOD_US) tNext = now + PERIOD_US;

  // One encoder read per tick; getAngle/getVelocity are cached after this.
  if (!left.update(DT)) {
    Serial.println("encoder read fail — stopping");
    left.disable();
    fallen = true;
    return;
  }

  float accAngle, gyroRate;
  if (!readMPU(accAngle, gyroRate)) {
    Serial.println("MPU read fail — stopping");
    left.disable();
    fallen = true;
    return;
  }

  tilt = 0.98f * (tilt + gyroRate * DT) + 0.02f * accAngle;

  float error  = BALANCE_POINT - tilt;
  float output = constrain(kp * error - kd * gyroRate, -MAX_VOLTS, MAX_VOLTS);

  if (fabsf(tilt - BALANCE_POINT) > FALL_LIMIT) {
    Serial.println("fallen — stopping");
    left.disable();
    fallen = true;
    return;
  }
  if (MOTOR_ENABLED) left.setTorque(output);

  static uint32_t lp = 0;
  if (millis() - lp > 100) {
    lp = millis();
    Serial.printf("tilt %+.2f  gyro %+.2f  out %+.2f  joint %+.3f\n",
                  tilt, gyroRate, output, left.getAngle());
  }
}
