
#ifndef MARGAY_h
#define MARGAY_h

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <DS3231_Logger.h>
#include <MCP3421.h>
#include <SdFat.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <Arduino.h>
#include <MCP3421.h>


#define RED 0xFFFF0000L
#define GREEN 0xFF00FF00L
#define BLUE 0xFF0000FFL
#define MAROON 0xFF800000L
#define GOLD 0xFFFFD700L
#define ORANGE 0xFFFFA500L
#define PURPLE 0xFF800080L
#define CYAN 0xFF00FFFF
#define BLACK_ALERT 0x802019FF
#define OFF 0x00

#define MODEL_1v0 
#define MODEL_0v0

enum board
{
    Model_0v0 = 0,
    Model_1v0 = 1
};

////////////////////////////PIN DEFINITIONS///////////////////////

class Margay
{

	public:
		Margay(board Model);
		int begin(uint8_t *Vals, uint8_t NumVals, String Header_);
		int begin(String Header_);

		int LogStr(String Val);
		void LED_Color(unsigned long Val);
		void Run(String (*f)(void), unsigned long LogInterval);
		float GetVoltage();
		void AddDataPoint(String (*Update)(void));

		//Pin definitions
		int SD_CS = 4;
		uint8_t BuiltInLED = 20;
		uint8_t RedLED = 13;
		uint8_t GreenLED = 15;
		uint8_t BlueLED = 14;

		uint8_t VRef_Pin = 2;
		uint8_t ThermSense_Pin = 1;
		uint8_t BatSense_Pin = 0;

		uint8_t VSwitch_Pin = 3;
		uint8_t SD_CD = 1;

		uint8_t Ext3v3Ctrl = 19;
		uint8_t I2C_SW = 12;
		uint8_t PG = 18;
		uint8_t ExtInt = 11;
		uint8_t RTCInt = 10;
		uint8_t LogInt = 2; 

	protected:
		float TempConvert(float V, float Vcc, float R, float A, float B, float C, float D, float R25);
		void Blink();
		// void StartLog();
		// void Log();
		void virtual Log();
		void virtual ButtonLog();
		static void isr0();
		static void isr1();
		static Margay* selfPointer;

		void sleepNow();
		void turnOffSDcard();
		void turnOnSDcard();
		void GetTime();
		String GetOnBoardVals();
		void I2CTest();
		void SDTest();
		void ClockTest();
		void PowerTest();
		void InitLogFile();

		DS3231_Logger RTC;
		MCP3421 adc;

		float A = 0.003354016;
		float B = 0.0003074038;
		float C = 1.019153E-05;
		float D = 9.093712E-07;
		String LogTimeDate = "2063/04/05 20:00:00";
		// float Temp[5] = {0}; //Temp Downhole, Temp on board, Temp RTC, Temp Baro
		// float Pressure[2] = {0}; //Downhole pressure, Atmospheric pressure (if applicable)
		float BatVoltage = 0;
		bool OBError = false;
		bool SensorError = false;
		bool TimeError = false;
		bool SDError = false; //USE??
		String Header = "";
		uint8_t NumADR = 0;
		uint8_t I2C_ADR[16] = {0}; //Change length??
		 uint8_t NumADR_OB = 2;
		 uint8_t I2C_ADR_OB[2] = {0x68, 0x6A}; //ADC, Clock

		volatile bool LogEvent = false; //Used to test if logging should begin yet
		volatile bool NewLog = false; //Used to tell system to start a new log
		volatile bool ManualLog = false; //Used to add point to log by pressing the log button
		volatile int AwakeCount = 0;

		char FileNameC[11]; //Used for file handling
		char FileNameTestC[11]; //Used for file handling
		bool SD_Init = false;
		SdFat SD;
		byte  keep_SPCR;
};

#endif