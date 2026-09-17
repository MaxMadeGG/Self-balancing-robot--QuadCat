#include "GimbalMotor.h"

static const float TWO_PI_F  = 6.28319f;
static const float PI_F      = 3.14159f;
static const float HALF_PI_F = 1.5708f;
static const float PHASE_B   = 2.09440f;
static const float PHASE_C   = 4.18879f;
static const int   PWM_FREQ  = 20000;
static const int   PWM_RES   = 8;

static const uint8_t AS5600_ADDR   = 0x36;
static const uint8_t AS5600_ANGLE  = 0x0C;
static const uint8_t AS5600_STATUS = 0x0B;
static const uint8_t AS5600_AGC    = 0x1A;

GimbalMotor::GimbalMotor(int a, int b, int c, int en, int poles, TwoWire &bus)
  : pinA(a), pinB(b), pinC(c), pinEn(en), polePairs(poles), wire(&bus) {}

float GimbalMotor::normAngle(float a) {
  a = fmodf(a, TWO_PI_F);
  return (a >= 0) ? a : a + TWO_PI_F;
}

void GimbalMotor::init() {
  // EN low BEFORE attaching PWM: GPIO5 and GPIO14 output a burst at boot,
  // and the driver must not be live with undefined inputs.
  pinMode(pinEn, OUTPUT);
  digitalWrite(pinEn, LOW);
  ledcAttach(pinA, PWM_FREQ, PWM_RES);
  ledcAttach(pinB, PWM_FREQ, PWM_RES);
  ledcAttach(pinC, PWM_FREQ, PWM_RES);
  setPhase(0, 0);
}

void GimbalMotor::disable() {
  setPhase(0, 0);
  digitalWrite(pinEn, LOW);
  ready = false;
}

// -1.0 on any failure. A dead bus must never look like a stationary shaft.
// The direction inversion lives here so one convention holds everywhere.
float GimbalMotor::readAngleRaw() {
  wire->beginTransmission(AS5600_ADDR);
  wire->write(AS5600_ANGLE);
  if (wire->endTransmission(false) != 0) return -1.0f;
  if (wire->requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return -1.0f;
  if (wire->available() < 2) return -1.0f;
  int raw = ((wire->read() << 8) | wire->read()) & 0x0FFF;
  float a = TWO_PI_F - (raw * TWO_PI_F / 4096.0f);
  return normAngle(encDir > 0 ? a : -a);
}

int GimbalMotor::readReg(uint8_t reg) {
  wire->beginTransmission(AS5600_ADDR);
  wire->write(reg);
  if (wire->endTransmission(false) != 0) return -1;
  if (wire->requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1) return -1;
  return wire->read();
}

int GimbalMotor::encoderAGC()    { return readReg(AS5600_AGC); }
int GimbalMotor::encoderStatus() { return readReg(AS5600_STATUS); }

// dt must be CONSTANT. Measuring it per-iteration produced velocity spikes of
// +-19 rad/s on a stationary shaft, which KD then amplified into a limit cycle.
bool GimbalMotor::update(float dt) {
  float raw = readAngleRaw();
  if (raw < 0) return false;

  if (!started) { lastRaw = raw; fullAngle = raw; started = true; }

  float d = raw - lastRaw;
  if (d >  PI_F) d -= TWO_PI_F;
  if (d < -PI_F) d += TWO_PI_F;
  lastRaw    = raw;
  fullAngle += d;

  float vRaw = d / dt;
  velValue  += (vRaw - velValue) * velAlpha;
  return true;
}

void GimbalMotor::setPhase(float angle, float volts) {
  float ceiling = supply * 0.5f;
  if (volts > ceiling) volts = ceiling;
  if (volts < 0)       volts = 0;

  float amp = (volts / ceiling) * 127.0f;
  ledcWrite(pinA, constrain(128 + (int)(amp * sinf(angle)),           0, 255));
  ledcWrite(pinB, constrain(128 + (int)(amp * sinf(angle + PHASE_B)), 0, 255));
  ledcWrite(pinC, constrain(128 + (int)(amp * sinf(angle + PHASE_C)), 0, 255));
}

// Uses the cached angle from update() — no extra I2C read, and position,
// velocity and commutation all come from the same sample.
void GimbalMotor::setTorque(float volts) {
  if (volts >  voltageLimit) volts =  voltageLimit;
  if (volts < -voltageLimit) volts = -voltageLimit;

  float e    = normAngle(fullAngle * polePairs - zeroOffset);
  float lead = (volts >= 0) ? HALF_PI_F : -HALF_PI_F;
  setPhase(normAngle(e + lead), fabsf(volts));
}

bool GimbalMotor::calibrate(int elecRevs) {
  digitalWrite(pinEn, HIGH);
  setPhase(0, calVoltage);
  delay(1000);

  float prev = readAngleRaw();
  if (prev < 0) { disable(); return false; }

  const float step  = 0.01f;
  const int   steps = (int)(TWO_PI_F * elecRevs / step);
  float total = 0;

  for (int i = 0; i < steps; i++) {
    setPhase(normAngle(i * step), calVoltage);
    delayMicroseconds(500);
    float cur = readAngleRaw();
    if (cur < 0) { disable(); return false; }
    float d = cur - prev;
    if (d >  PI_F) d -= TWO_PI_F;
    if (d < -PI_F) d += TWO_PI_F;
    total += d;
    prev = cur;
  }

  lastMechRevs = total / TWO_PI_F;
  lastImplied  = elecRevs / fabsf(lastMechRevs);

  // Encoder counts backwards relative to the field sweep. Fix at the source
  // and sweep again — flipping the torque lead instead leaves the commutation
  // itself running backwards, which vibrates.
  if (lastMechRevs < 0 && encDir > 0) {
    encDir = -1;
    setPhase(0, 0);
    digitalWrite(pinEn, LOW);
    delay(500);
    return calibrate(elecRevs);
  }

  // The sanity check. Off by more than this and the rotor is not turning —
  // a dead phase, a wrong pin map, an encoder not tracking the shaft.
  if (fabsf(lastImplied - polePairs) > 0.5f) { disable(); return false; }

  setPhase(0, calVoltage);
  delay(1500);
  float a = readAngleRaw();
  if (a < 0) { disable(); return false; }

  zeroOffset = normAngle(a * polePairs);
  lastRaw    = a;
  fullAngle  = a;
  started    = true;
  velValue   = 0;

  setPhase(0, 0);
  digitalWrite(pinEn, LOW);
  ready = true;
  return true;
}
