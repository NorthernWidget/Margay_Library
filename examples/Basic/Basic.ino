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

// Empty header and I2C list: no external sensors
String header = "";
uint8_t I2CVals[] = {};

// Number of seconds between readings
uint32_t updateRate = 5;

Margay Logger(MODEL_3v0); // Update to match your hardware version

void setup() {
    Logger.begin(I2CVals, sizeof(I2CVals), header);
    initialize();
}

void loop() {
    Logger.run(update, updateRate);
}

String update() {
    initialize();
    return "";
}

void initialize() {
    // Place any sensor initialization calls here
}
