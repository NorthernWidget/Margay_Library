/******************************************************************************
BusScan.ino
Ask the bus what is attached, rather than telling the logger what to expect.

Every Schema 1 device carries its own name, versions and serial number in
Page 0, so a scan can name whatever answered. Run this to find out what is on
a board before writing a sketch for it, and to check that a sensor you have
just wired is answering at the address you think.

Nothing is watched and nothing is logged: this reports and stops.

Andy Wickert, Northern Widget LLC
10/3/2026
https://github.com/NorthernWidget/Margay_Library

Distributed as-is; no warranty is given.
******************************************************************************/

#include "Margay.h"

Margay Logger(MODEL_3v0); // Update to match your hardware version

void setup() {
    Logger.begin();
    Logger.scan(Serial);
}

void loop() {
}
