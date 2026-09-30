#!/bin/sh
# Builds and runs the charge-status ladder decoder tests.
set -e
cd "$(dirname "$0")"
BUILD_DIR="${TMPDIR:-/tmp}/freeink-battery-tests"
mkdir -p "$BUILD_DIR"
c++ -std=c++17 -Wall -Wextra -Werror test_charge_ladder_decoder.cpp -o "$BUILD_DIR/test_charge_ladder_decoder"
"$BUILD_DIR/test_charge_ladder_decoder"
