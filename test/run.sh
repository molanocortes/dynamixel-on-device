#!/usr/bin/env bash
# Host test suite for tiny_dxl. Compiles and runs the frame-codec tests and the
# full-sketch integration tests against an emulated two-servo Dynamixel bus.
# NO hardware and NO Arduino/Teensy toolchain required - just a C++17 compiler.
#
#   ./run.sh            # build + run both suites
#   CXX=g++ ./run.sh    # pick a compiler
set -e
cd "$(dirname "$0")"
CXX="${CXX:-c++}"

echo "== building codec tests (frame codec: Fast Sync Read parse, CRC, byte-stuffing) =="
"$CXX" -std=c++17 -O1 -Wall -I stub -I ../src test_codec.cpp -o test_codec

echo "== building integration tests (real sketch vs emulated bus) =="
"$CXX" -std=c++17 -O1 -I motorstub -I ../src test_motor.cpp -o test_motor

echo "== running =="
./test_codec
./test_motor
