//MargayDemo.ino
#include "Margay.h"
#include <TP_Downhole.h>

TP_Downhole Sensor; //Initalize TP-Downhole sensor

String Header = "Pressure [mBar],Temp DH [C], Temp DHt [C]"; //Information header
uint8_t I2CVals[2] = {0x6A, 0x77}; 
unsigned long UpdateRate = 5; //Number of seconds between readings 

Margay Logger(Model_0v0);

void setup() {
  Logger.begin(I2CVals, sizeof(I2CVals), Header); //Pass header info to logger
  Init();
}

void loop() {
  Logger.Run(Update, UpdateRate);
}

String Update() 
{
	float Val1 = Sensor.getPressure();
	float Val2 = Sensor.getTemperature(0);
	float Val3 = Sensor.getTemperature(1);
	return String(Val1) + "," + String(Val2) + "," + String(Val3); 
}

void Init() 
{
	Sensor.begin(TP2v2);
}
