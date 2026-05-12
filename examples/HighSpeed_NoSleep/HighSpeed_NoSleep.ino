/******************************************************************************
HighSpeed_NoSleep.ino
High-speed logging example without sleep/power-save mode.

Andy Wickert, Bobby Schulz @ Northern Widget LLC
https://github.com/NorthernWidget/Margay_Library

Logs onboard sensors (BME280 - temperature/pressure/humidity) as fast as
possible by polling in loop() rather than using the standard sleep-based
Logger.run(). Intended for sensor evaluation and calibration on the benchtop,
NOT for field deployment (high power draw).

Add external sensor reads inside update() and their I2C addresses to I2CVals
to extend this example.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "Margay.h"

String header = "";
uint8_t I2CVals[] = {};

uint32_t updateRate = 150; // Milliseconds between readings

Margay Logger(MODEL_3v0); // Update to match your hardware version

void setup() {
    Logger.begin(I2CVals, sizeof(I2CVals), header);
    Logger.initLogFile(); // Generate a new log file on each reset
    initialize();
}

void loop() {
    static uint32_t trigger = millis();
    if (millis() - trigger > updateRate) {
        trigger = millis();
        Logger.LED_Color(BLUE);
        Logger.addDataPoint(update);
        Logger.LED_Color(OFF);
    }
}

String update() {
    initialize();
    return "";
}

void initialize() {
    // Place any sensor initialization calls here
}
