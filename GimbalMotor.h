#pragma once
#include <Arduino.h>
#include <Wire.h>

class GimbalMotor {
public:
  GimbalMotor(int a, int b, int c, int en, int poles, TwoWire &bus = Wire);

  void  init();                          // EN low first: GPIO5/14 glitch at boot
  void  disable();

  // Open-loop sweep: detects encoder direction, checks implied pole pairs,
  // derives the offset. False means the rotor did not turn — hardware fault,
  // and any offset that followed would be meaningless.
  bool  calibrate(int elecRevs = 20);

  // ONE I2C read per control tick. dt must be CONSTANT — a measured dt gave
  // velocity spikes of +-19 rad/s on a still shaft, which KD turned into a
  // limit cycle. False on encoder failure: stop the motor, do not continue.
  bool  update(float dt);

  void  setTorque(float volts);          // signed, uses the cached angle

  float getAngle()     const { return fullAngle; }   // unwrapped, radians
  float getVelocity()  const { return velValue; }
  bool  isReady()      const { return ready; }
  float getOffset()    const { return zeroOffset; }
  float getMechRevs()  const { return lastMechRevs; }
  float getImplied()   const { return lastImplied; }
  int   getEncDir()    const { return encDir; }

  int   encoderAGC();                    // ~120 good, 255 too far, 0 too close
  int   encoderStatus();                 // bit5 MD, bit4 ML, bit3 MH

  void  setSupply(float v)       { supply = v; }
  void  setVoltageLimit(float v) { voltageLimit = v; }
  void  setCalVoltage(float v)   { calVoltage = v; }
  void  setVelAlpha(float a)     { velAlpha = a; }
  void  enable()                 { digitalWrite(pinEn, HIGH); }

private:
  int pinA, pinB, pinC, pinEn;
  int polePairs;
  TwoWire *wire;

  float supply       = 10.0f;
  float voltageLimit = 4.0f;
  float calVoltage   = 8.0f;             // above voltageLimit on purpose
  float velAlpha     = 0.05f;            // tuned bare-shaft

  float zeroOffset   = 0;
  int   encDir       = 1;                // set by calibrate()
  bool  ready        = false;

  float fullAngle    = 0;                // unwrapped
  float lastRaw      = 0;
  bool  started      = false;
  float velValue     = 0;

  float lastMechRevs = 0;
  float lastImplied  = 0;

  float readAngleRaw();                  // -1.0 on ANY failure, never stale
  int   readReg(uint8_t reg);
  void  setPhase(float angle, float volts);
  static float normAngle(float a);
};
