/******************************************************************************
Basic.ino
Most basic logger example.
Reports only onboard sensors (BME280 - temperature/pressure/humidity) and device diagnostics, does so every 5 seconds

Bobby Schulz @ Northern Widget LLC
8/8/2024
https://github.com/NorthernWidget-Skunkworks/Margay_Library

Intended as a first test program to run. 

Distributed as-is; no warranty is given.
******************************************************************************/

#include "Margay.h"

// String Header = ""; //Information header, name of each reading seperated by commas 
// uint8_t I2CVals[1] = {}; //List of I2C values to look for (other than on board ones)

unsigned long UpdateRate = 5; //Number of seconds between readings 

Margay Logger(MODEL_2v2); //Define which version of Margay is being used

void setup() {
  Logger.begin(); //Pass header info to logger
  // Init(); //Call initialization if other sensors are used
}

void loop() {
  Logger.run(Update, UpdateRate);
}

String Update() 
{
	//Call sensor readings here
	return ""; //Append readings together, sperated by commas, here
}

void Init() 
{
  //Put sensors initializations here
}
