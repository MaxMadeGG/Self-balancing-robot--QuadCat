#pragma once
#include <Arduino.h>
#include <Wire.h>

class GimbalMotor {
public:
  GimbalMotor(int a, int b, int c, int en, int poles, TwoWire &bus = Wire);

  void  init();
  void  calibrate();
  void  setTorque(float volts);
  void  disable();

  float readAngle();
  float readFullAngle();
  float getVelocity();

  void  setSupply(float v)       { supply = v; }
  void  setVoltageLimit(float v) { voltageLimit = v; }
  float getOffset()              { return zeroOffset; }

private:
  int pinA, pinB, pinC, pinEn;
  int polePairs;
  TwoWire *wire;

  float supply       = 10.0f;
  float voltageLimit = 5.0f;
  float zeroOffset   = 0;

  float lastGood  = 0;
  float prevWrap  = 0;
  long  rotations = 0;
  bool  started   = false;

  float velPrev  = 0;
  float velValue = 0;
  unsigned long velLastT = 0;

  void setPhase(float angle, float volts);
  static float normAngle(float a);
};