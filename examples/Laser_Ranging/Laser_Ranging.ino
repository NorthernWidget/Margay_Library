/******************************************************************************
Laser_Ranging.ino
Margay data logger with Apis LiDAR laser rangefinder.

Andy Wickert @ Northern Widget LLC
https://github.com/NorthernWidget/Margay_Library

Logs range [cm], pitch [deg], and roll [deg] from the Apis LiDAR module
once per minute. The Apis re-initializes on each logging cycle to recover
from occasional LiDAR Lite firmware hangs.

Requires the Apis library: https://github.com/NorthernWidget/Apis_Library

Distributed as-is; no warranty is given.
******************************************************************************/

#include "Margay.h"
#include "Apis.h"

Margay Logger(MODEL_3v0); // Update to match your hardware version
Apis rangefinder;

uint8_t I2CVals[] = {ADR_DEFAULT};
String header = "";
uint32_t updateRate = 60; // Seconds between readings

void setup() {
    header = rangefinder.getHeader();
    Logger.begin(I2CVals, sizeof(I2CVals), header);
    initialize();
}

void loop() {
    Logger.run(update, updateRate);
}

String update() {
    initialize();
    return rangefinder.getString();
}

void initialize() {
    rangefinder.begin();
}
