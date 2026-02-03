#!/bin/bash
# Build and run tests for a library.
# 
# Either copy this file into the root of your library and run it to build and execute tests
# or set the LIB_DIR environment variable to point to your library before running this script.
#

set -e

if [ -z "$LIB_DIR" -o ! -d "$LIB_DIR" ]; then
    LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi

export ARDUINO_DIRECTORIES_DATA="$LIB_DIR/build/.arduino"
export ARDUINO_DIRECTORIES_DOWNLOADS="$LIB_DIR/build/.arduino/staging"
export ARDUINO_DIRECTORIES_USER="$LIB_DIR/build/.arduino"

if [ -z "$YATEST_DIR" ]; then
    YATEST_DIR="$(cd "$ARDUINO_DIRECTORIES_USER/libraries/yatest" && pwd)"
    if [ ! -d "$YATEST_DIR" -o "$YATEST_FORCE_INSTALL" -eq 1 ]; then
        echo "Yatest not found or forced to install. Installing yatest library..."
        arduino-cli lib update-index
        arduino-cli lib install yatest
    fi
fi

if [ ! -f "$YATEST_DIR/src/build-and-run.sh" ]; then
    echo "Error: yatest build script not found at $YATEST_DIR"
    exit 1
fi

"$YATEST_DIR/src/build-and-run.sh" "$LIB_DIR"
