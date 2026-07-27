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

# Same suite pretending the core drives the direction pin from the UART ISR
# (what Teensy's transmitterEnable does). Same code path as the default build
# above, just resolving to a different strategy. Without this the hardware
# branch is #if'd out on the host and could rot unnoticed until someone flashed
# a Teensy.
echo "== building codec tests as a hardware-direction core (Teensy) =="
"$CXX" -std=c++17 -O1 -Wall -DTINY_DXL_HAS_HW_DIR=1 -I stub -I ../src test_codec.cpp -o test_codec_hwdir

# The two opt-in pinned builds. Nobody needs these, but they are supported, so
# they have to keep compiling and passing.
echo "== building codec tests with the mode pinned to HARDWARE (opt-in) =="
"$CXX" -std=c++17 -O1 -Wall -DTINY_DXL_HAS_HW_DIR=1 -DTINY_DXL_DIR_MODE=DXL_DIR_HARDWARE \
  -I stub -I ../src test_codec.cpp -o test_codec_pinhw

echo "== building codec tests with the mode pinned to MANUAL (opt-in) =="
"$CXX" -std=c++17 -O1 -Wall -DTINY_DXL_DIR_MODE=DXL_DIR_MANUAL \
  -I stub -I ../src test_codec.cpp -o test_codec_pinned

echo "== building integration tests (real sketch vs emulated bus) =="
"$CXX" -std=c++17 -O1 -I motorstub -I ../src test_motor.cpp -o test_motor

echo "== running =="
./test_codec
./test_codec_hwdir
./test_codec_pinhw
./test_codec_pinned
./test_motor
