/******************************************************************************
Basic.ino
Most basic logger example.
Reports only onboard sensors (BME280 - temperature/pressure/humidity) and
device diagnostics. Intended as a first test program to run.

Andy Wickert, Bobby Schulz @ Northern Widget LLC
8/8/2024
https://github.com/NorthernWidget/Margay_Library

Distributed as-is; no warranty is given.
******************************************************************************/

#include "Margay.h"

// Number of seconds between readings
uint32_t updateRate = 5;

Margay Logger(MODEL_3v0); // Update to match your hardware version

void setup() {
    // Nothing watched: this logger reports only its own columns.
    Logger.begin();
}

void loop() {
    Logger.run(updateRate);
}
