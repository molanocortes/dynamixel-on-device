// Minimal SD stub (compile-satisfies bringup_12ch.ino). Not exercised at runtime
// in the motor test (recording stays off).
#pragma once
#include <Arduino.h>
#define BUILTIN_SDCARD 254
#define FILE_WRITE 1

struct File {
  bool _open = false;
  operator bool() const { return _open; }
  void print(const char*) {}
  void print(char) {}
  int  printf(const char*, ...) { return 0; }
  void println(const char*) {}
  void close() { _open = false; }
  size_t write(uint8_t) { return 1; }
};

struct SDClass {
  bool begin(uint8_t) { return false; }
  bool exists(const char*) { return false; }
  File open(const char*, uint8_t) { return File(); }
};

static SDClass SD;
