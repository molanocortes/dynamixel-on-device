// Minimal TwoWire stub (compile-satisfies bringup_12ch.ino + tiny_bno085.h).
// The motor test never drives I2C at runtime, so returns are just benign.
#pragma once
#include <Arduino.h>

struct TwoWire {
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  size_t write(const uint8_t*, size_t n) { return n; }
  uint8_t endTransmission() { return 0; }
  uint8_t endTransmission(bool) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t n) { return n; }
  int     requestFrom(int, int n) { return n; }
  int     available() { return 0; }
  int     read() { return 0; }
};

static TwoWire Wire, Wire1, Wire2;
