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

// Number of seconds between readings
uint32_t updateRate = 60;

Margay Logger(MODEL_3v0); // Update to match your hardware version
Apis rangefinder;

void setup() {
    // One line states the sensor, its address and its column order. The logger
    // writes the header and every row from it; this sketch composes nothing.
    Logger.watch(rangefinder);
    Logger.begin();
}

void loop() {
    Logger.run(updateRate);
}
