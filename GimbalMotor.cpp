#include "GimbalMotor.h"

static const float TWO_PI_F  = 6.28319f;
static const float PI_F      = 3.14159f;
static const float HALF_PI_F = 1.5708f;
static const int   PWM_FREQ  = 20000;
static const int   PWM_RES   = 8;

GimbalMotor::GimbalMotor(int a, int b, int c, int en, int poles, TwoWire &bus)
  : pinA(a), pinB(b), pinC(c), pinEn(en), polePairs(poles), wire(&bus) {}

float GimbalMotor::normAngle(float a) {
  a = fmodf(a, TWO_PI_F);
  return (a >= 0) ? a : a + TWO_PI_F;
}

void GimbalMotor::init() {
  ledcAttach(pinA, PWM_FREQ, PWM_RES);
  ledcAttach(pinB, PWM_FREQ, PWM_RES);
  ledcAttach(pinC, PWM_FREQ, PWM_RES);
  pinMode(pinEn, OUTPUT);
  digitalWrite(pinEn, HIGH);
  setPhase(0, 0);
  velLastT = micros();
}

void GimbalMotor::disable() {
  setPhase(0, 0);
  digitalWrite(pinEn, LOW);
}

float GimbalMotor::readAngle() {
  wire->beginTransmission(0x36);
  wire->write(0x0C);
  wire->endTransmission(false);
  wire->requestFrom(0x36, 2);
  if (wire->available() < 2) return lastGood;

  int raw = ((wire->read() << 8) | wire->read()) & 0x0FFF;
  lastGood = raw * TWO_PI_F / 4096.0f;
  return lastGood;
}

float GimbalMotor::readFullAngle() {
  float a = readAngle();
  if (!started) { prevWrap = a; started = true; }

  float d = a - prevWrap;
  if (d >  PI_F) rotations--;
  if (d < -PI_F) rotations++;
  prevWrap = a;

  return a + (float)rotations * TWO_PI_F;
}

float GimbalMotor::getVelocity() {
  unsigned long now = micros();
  float dt = (now - velLastT) * 1e-6f;
  if (dt < 0.0005f) return velValue;
  velLastT = now;

  float a = readAngle();
  float d = a - velPrev;
  if (d >  PI_F) d -= TWO_PI_F;
  if (d < -PI_F) d += TWO_PI_F;
  velPrev = a;

  float raw = d / dt;
  velValue = 0.8f * velValue + 0.2f * raw;
  return velValue;
}

void GimbalMotor::setPhase(float angle, float volts) {
  if (volts > voltageLimit) volts = voltageLimit;
  if (volts < 0) volts = 0;

  float amp = (volts / (supply * 0.5f)) * 127.0f;

  int da = 128 + (int)(amp * sinf(angle));
  int db = 128 + (int)(amp * sinf(angle + 2.09440f));
  int dc = 128 + (int)(amp * sinf(angle + 4.18879f));

  ledcWrite(pinA, constrain(da, 0, 255));
  ledcWrite(pinB, constrain(db, 0, 255));
  ledcWrite(pinC, constrain(dc, 0, 255));
}

void GimbalMotor::calibrate() {
  setPhase(0, voltageLimit * 0.6f);
  delay(1500);
  zeroOffset = normAngle(readAngle() * polePairs);
  setPhase(0, 0);
  delay(300);

  velPrev  = readAngle();
  prevWrap = velPrev;
  velLastT = micros();
}

void GimbalMotor::setTorque(float volts) {
  float e    = normAngle(readAngle() * polePairs - zeroOffset);
  float lead = (volts >= 0) ? HALF_PI_F : -HALF_PI_F;
  setPhase(normAngle(e + lead), fabsf(volts));
}