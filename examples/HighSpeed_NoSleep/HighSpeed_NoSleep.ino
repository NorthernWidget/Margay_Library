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

uint32_t updateRate = 150; // Milliseconds between readings

Margay Logger(MODEL_3v0); // Update to match your hardware version

void setup() {
    // Watch any external sensor here; the logger takes its address and its
    // columns from that one line.
    Logger.begin();
    Logger.initLogFile(); // Generate a new log file on each reset
}

void loop() {
    // No sleeping and no RTC alarm: this loop times the readings itself and
    // asks for a row directly, which is why it is milliseconds rather than
    // seconds and why run() is not used.
    static uint32_t trigger = millis();
    if (millis() - trigger > updateRate) {
        trigger = millis();
        Logger.LED_Color(BLUE);
        Logger.addDataPoint();
        Logger.LED_Color(OFF);
    }
}
