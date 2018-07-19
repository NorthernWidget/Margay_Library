//Margay library

#include <SPI.h>
#include <Wire.h>
#include <DS3231_Logger.h>
#include <SdFat.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <Margay.h>
#include <MCP3421.h>
// #include <Arduino.h>

Margay* Margay::selfPointer;

Margay::Margay(board Model)
{
if(Model == 1) {
	SD_CS = 4;
	BuiltInLED = 20;
	RedLED = 13;
	GreenLED = 15;
	BlueLED = 14;

	VRef_Pin = 2;
	ThermSense_Pin = 1;
	BatSense_Pin = 0;

	VSwitch_Pin = 3;
	SD_CD = 1;

	Ext3v3Ctrl = 19;
	I2C_SW = 12;
	PG = 18;
	ExtInt = 11;
	RTCInt = 10;
	LogInt = 2; 
}
else {
	SD_CS = 4;
	BuiltInLED = 19;
	RedLED = 13;
	GreenLED = 15;
	BlueLED = 14;

	VRef_Pin = 2;
	ThermSense_Pin = 1;
	BatSense_Pin = 0;

	VSwitch_Pin = 3;
	SD_CD = 1;

	Ext3v3Ctrl = 12;
	I2C_SW = 255;
	PG = 18;
	ExtInt = 11;
	RTCInt = 10;
	LogInt = 2; 
}
}

int Margay::begin(uint8_t *Vals, uint8_t NumVals, String Header_)
{
	pinMode(Ext3v3Ctrl, OUTPUT);
	digitalWrite(Ext3v3Ctrl, LOW); //Make sure external power is on

	pinMode(BuiltInLED, OUTPUT);
	digitalWrite(BuiltInLED, LOW); //Turn built in LED on

	memcpy(I2C_ADR, Vals, sizeof(I2C_ADR)); //Copy array
	NumADR = NumVals; //Copy length of array
	Header = Header_; //Copy user defined header

	RTC.Begin(); //Initalize RTC
	RTC.ClearAlarm(); //
	adc.Begin(); //Initalize external ADC
	adc.SetResolution(18);

	Serial.begin(38400); //DEBUG!
	Serial.println("\nInitializing...\n"); //DEBUG!
	delay(100);
	if(Serial.available()) {  //If time setting info available 
		Serial.println("BANG!"); //DEBUG!
		String DateTimeTemp = Serial.readString();
		Serial.println(DateTimeTemp);  //DEBUG!
		int DateTimeVals[6] = {0};
		for(int i = 0; i < 6; i++) {
			DateTimeVals[i] = DateTimeTemp.substring(2*i, 2*(i+1)).toInt();
			Serial.print(i); Serial.print("  "); Serial.println(DateTimeVals[i]);  //DEBUG!
		}
		RTC.SetTime(2000 + DateTimeVals[0], DateTimeVals[1], DateTimeVals[2], DateTimeVals[3], DateTimeVals[4], DateTimeVals[5]);
	}

	//Sets up basic initialization required for the system
	selfPointer = this;


	pinMode(RedLED, OUTPUT);
	pinMode(GreenLED, OUTPUT);
	pinMode(BlueLED, OUTPUT);

	LED_Color(OFF);

	// Wire.begin();
	pinMode(SD_CS, OUTPUT);
	// SPI.setDataMode(SPI_MODE0);
	// SPI.setClockDivider(SPI_CLOCK_DIV2); //Sts SPI clock to 4 MHz for an 8 MHz system clock


	attachInterrupt(digitalPinToInterrupt(RTCInt), Margay::isr1, FALLING); //Attach an interrupt driven by the interrupt from RTC, logs data
	attachInterrupt(digitalPinToInterrupt(LogInt), Margay::isr0, FALLING);	//Attach an interrupt driven by the manual log button, sets logging flag and logs data
	pinMode(RTCInt, INPUT_PULLUP);
	pinMode(LogInt, INPUT);

	I2CTest();
	ClockTest();
	SDTest();
	Serial.print("\nTimestamp = ");
	Serial.println(LogTimeDate);
  	
  	digitalWrite(BuiltInLED, HIGH); 

  	if(OBError) {
  		LED_Color(RED);	//On board failure
  		delay(2000);
  	}
	if(SensorError) {
		LED_Color(ORANGE);  //Sensor failure
		delay(2000);
	}
	if(TimeError) {
		LED_Color(CYAN); //Time set error
		delay(2000);
	}
	if(SDError) {
		LED_Color(PURPLE); //Sd card not inserted
		delay(2000);
	}
	if(!OBError && !SensorError && !TimeError && !SDError) {
		LED_Color(GREEN); 
		delay(2000);
	}

	Serial.print("\nReady to Log...\n\n");
	NewLog = true; //Set flag to begin new log file

	// delay(2000);

	LED_Color(OFF);
}

int Margay::begin(String Header_)
{
	uint8_t Dummy = {NULL};
	begin(Dummy, 0, Header_); //Call generalized begin function
}

void Margay::I2CTest() 
{
	int Error = 0;
	bool I2C_Test = true;

	Serial.print("I2C: ");

	for(int i = 0; i < NumADR; i++) {
		Wire.beginTransmission(I2C_ADR[i]);
    	Error = Wire.endTransmission();
    	if(Error != 0) {
    		if(I2C_Test) Serial.println(" Fail");
    		Serial.print("   Fail At: ");
    		Serial.println(I2C_ADR[i], HEX);
    		I2C_Test = false;
    		SensorError = true;
		}
	}

	//Switch to connect to onboard I2C!

	for(int i = 0; i < NumADR_OB; i++) {
		Wire.beginTransmission(I2C_ADR_OB[i]);
    	Error = Wire.endTransmission();
    	if(Error != 0) {
    		if(I2C_Test) Serial.println(" Fail");
    		Serial.print("   Fail At: ");
    		Serial.println(I2C_ADR_OB[i], HEX);
    		I2C_Test = false;
    		OBError = true;
		}
	}

	if(I2C_Test) Serial.println("PASS");
}


void Margay::SDTest() 
{
	bool SDErrorTemp = false;
	// bool SD_Test = true;

	pinMode(SD_CD, INPUT);
	bool CardPressent = digitalRead(SD_CD);

	Serial.print("SD: ");

	if(CardPressent) {
		Serial.println(" NO CARD");
    	SDErrorTemp = true;
    	SDError = true; //Card not inserted
	}

	else if (!SD.begin(SD_CS)) {
    	OBError = true;
    	SDErrorTemp = true;
  	}

  	if(!CardPressent) {
		String FileNameTest = "HWTest";
		(FileNameTest + ".txt").toCharArray(FileNameTestC, 11);
		SD.remove(FileNameTestC); //Remove any previous files

		randomSeed(analogRead(A7)); //Seed with a random number to try to endsure randomness
		int RandVal = random(30557); //Generate a random number between 0 and 30557 (the number of words in Hamlet)
		char RandDigits[6] = {0};
		sprintf(RandDigits, "%d", RandVal); //Convert RandVal into a series of digits
		int RandLength = (int)((ceil(log10(RandVal))+1)*sizeof(char)); //Find the length of the values in the array

		File DataWrite = SD.open(FileNameTestC, FILE_WRITE);
		if(DataWrite) {
		DataWrite.println(RandVal);
		DataWrite.println("\nHe was a man. Take him for all in all.");
		DataWrite.println("I shall not look upon his like again.");
		DataWrite.println("-Hamlet, Act 1, Scene 2");
		}
		DataWrite.close();

		char TestDigits[6] = {0};
		File DataRead = SD.open(FileNameTestC, FILE_READ);
		if(DataRead) {
		DataRead.read(TestDigits, RandLength);

		for(int i = 0; i < RandLength - 1; i++){ //Test random value string
		  if(TestDigits[i] != RandDigits[i]) {
		    SDErrorTemp = true;
		    OBError = true;
		  }
		}
		}
		DataRead.close();

		keep_SPCR=SPCR; 
	}
  
	if(SDError && !CardPressent) Serial.println("FAIL");
  	else if(!SDError && !CardPressent) Serial.println("PASS");
}

void Margay::ClockTest() 
{ 
	int Error = 1;
	uint8_t TestSeconds = 0;
	bool OscStop = false;

	Serial.print("Clock: ");
	Wire.beginTransmission(I2C_ADR_OB[0]);
  	Wire.write(0xFF);
	Error = Wire.endTransmission();
	// Serial.print("Error = "); Serial.println(Error); //DEBUG!
	if(Error == 0) {
		GetTime(); //FIX!
		TestSeconds = RTC.GetValue(5);
	  	delay(1100);
	  	if(RTC.GetValue(5) == TestSeconds) {
	  		OBError = true; //If clock is not incrementing 
	  		OscStop = true; //Oscilator not running
	  	}
	}

	unsigned int YearNow = RTC.GetValue(0);
	// Serial.println(YearNow); //DEBUG!
	if(YearNow == 00) {  //If value is 2000, work around Y2K bug by setting time to Jan 1st, midnight, 2049
		// if(YearNow <= 00) RTC.SetTime(2018, 01, 01, 00, 00, 00);  //Only reset if Y2K
		// GetTime(); //Update local time
		TimeError = true;
		Serial.println(" PASS, BAD TIME");
	}

	if(Error != 0) {
		Serial.println(" FAIL");
		OBError = true; 
	}

	else if(Error == 0 && OscStop == false && TimeError == false) {
		Serial.println(" PASS");
	}
}

void Margay::PowerTest() 
{
	int Error = 0;

	digitalWrite(Ext3v3Ctrl, HIGH); //Turn off power to outputs

	Serial.print("Power: ");
	Wire.beginTransmission(I2C_ADR[1]);
	Error = Wire.endTransmission();
	if(Error == 0) Serial.println(" FAIL");

	if(Error != 0) Serial.println(" PASS");

	digitalWrite(Ext3v3Ctrl, LOW); //Turn power back on
}

void Margay::InitLogFile()
{
    String FileName = "Log";
    int FileNum = 1;
    String NumString = "01";
    (FileName + "01"+ ".txt").toCharArray(FileNameC, 11);
    while(SD.exists(FileNameC)) {
      FileNum += 1;
      NumString = String(FileNum, DEC);
      (FileName + NumString + ".txt").toCharArray(FileNameC, 11);
    }
    (FileName + NumString + ".txt").toCharArray(FileNameC, 11);
  
    LogStr("Time [UTC],Temp OB [C],Temp RTC [C],Bat [V], " + Header); //Log concatonated header
}

int Margay::LogStr(String Val) 
{
	Serial.println(Val); //Echo to serial monitor 
	SD.begin(SD_CS); //DEBUG!
	File DataFile = SD.open(FileNameC, FILE_WRITE);

	// if the file is available, write to it:
	if (DataFile) {
		DataFile.println(Val);
	   // return 0;
	}
	// if the file isn't open, pop up an error:
	else {
	   // return -1;
	}

	DataFile.close();
}

void Margay::LED_Color(unsigned long Val) //Set color of onboard led
{
	int Red = 0; //Red led color
	int Green = 0;  //Green led color
	int Blue = 0;  //Blue led color
	int Lum = 0;  //Luminosity

	//Parse all values from single Val
	Blue = Val & 0xFF;
	Green = (Val >> 8) & 0xFF;
	Red = (Val >> 16) & 0xFF;
	Lum = (Val >> 24) & 0xFF;
	//  Lum = 255 - Lum; //Invert since LEDs are open drain

	analogWrite(RedLED, 255 - (Red * Lum)/0xFF);
	analogWrite(GreenLED, 255 - (Green * Lum)/0xFF);
	analogWrite(BlueLED, 255 - (Blue * Lum)/0xFF);
}

void Margay::GetTime() 
{
	//Update global time string
	// DateTime TimeStamp = RTC.now();
	// LogTimeDate = String(TimeStamp.year()) + "/" + String(TimeStamp.month()) + "/" + String(TimeStamp.day()) + " " + String(TimeStamp.hour()) + ":" + String(TimeStamp.minute()) + ":" + String(TimeStamp.second());  
	LogTimeDate = RTC.GetTime(0);
}

String Margay::GetOnBoardVals() 
{
	//Get onboard temp, RTC temp, and battery voltage, referance voltage
	// float VRef = analogRead(VRef_Pin);
	float Vcc = 3.3; //(1.8/VRef)*3.3; //Compensate for Vcc using VRef
	// Serial.println(Vcc); //DEBUG!


	float Val = float(analogRead(ThermSense_Pin))*(Vcc/1024.0);
	//  float Vout = Vcc - Val;
	//  Serial.println(Val); //DEBUG!
	//  Serial.println(Vout);  //DEBUG!
	float TempData = TempConvert(Val, Vcc, 10000.0, A, B, C, D, 10000.0);
	TempData = TempData - 273.15; //Get temp from on board thermistor 

	BatVoltage = analogRead(BatSense_Pin)*9.0*(Vcc/1024.0); //Get battery voltage, Include voltage divider in math

	// Temp[3] = Clock.getTemperature(); //Get tempreture from RTC //FIX!
	float RTCTemp = RTC.GetTemp();
	GetTime(); //FIX!
	return  LogTimeDate + "," + String(TempData) + "," + String(RTCTemp) + "," + String(BatVoltage) + ",";
}

float Margay::TempConvert(float V, float Vcc, float R, float A, float B, float C, float D, float R25)
{
	//  Serial.print("R = "); //DEBUG!
	//  Serial.println(R); //DEBUG!
	float Rt = ((Vcc/V)*R) - R;
	//  Serial.print("Rt = "); //DEBUG!
	//  Serial.println(Rt); //DEBUG!
	float LogRt = log(Rt/R25);
	//  Serial.print("LogRt = "); //DEBUG!
	//  Serial.println(LogRt); //DEBUG!
	float T = 1.0/(A + B*LogRt + C*pow(LogRt, 2.0) + D*pow(LogRt, 3.0));
	return T;
}

void Margay::Blink() 
{  
  for(int i = 0; i < 5; i++) {
    digitalWrite(BlueLED, LOW);
    delay(500);
    digitalWrite(BlueLED, HIGH);
    delay(500);
  }
}

float Margay::GetVoltage()  //Get voltage from Ax pin
{
	float Val = adc.GetVoltage();
	return Val;
}

void Margay::Run(String (*Update)(void), unsigned long LogInterval) //Pass in function which returns string of data
{
	if(NewLog) {
		Serial.println("Log Started!"); //DEBUG
		// LogEvent = true;
		// unsigned long TempLogInterval = LogInterval; //ANDY, Fix with addition of function??
		RTC.SetAlarm(LogInterval); //DEBUG!
		InitLogFile(); //Start a new file each time log button is pressed

		//Add inital data point 
		AddDataPoint(Update);
		NewLog = false;  //Clear flag once log is started 
    	Blink();  //Alert user to start of log
	}

	if(LogEvent) {
		// Serial.println("Log!"); //DEBUG!
		AddDataPoint(Update); //Write values to SD
		LogEvent = false; //Clear log flag
		// Serial.println("BANG!"); //DEBUG!
		RTC.SetAlarm(LogInterval);  //Set/reset alarm
	}

	if(ManualLog) {  //Write data to SD card without interrupting existing timing cycle
		// Serial.println("Click!"); //DEBUG!
		AddDataPoint(Update); //write values to SD
		ManualLog = false; //Clear log flag
	}

	if(!digitalRead(RTCInt)) {  //Catch alarm if not reset properly 
   		Serial.println("Reset Alarm"); //DEBUG!
		RTC.SetAlarm(LogInterval); //Turn alarm back on 
	}

	AwakeCount++;

	if(AwakeCount > 5) {
	//    AwakeCount = 0;
		sleepNow();
	}
	delay(1);
}

void Margay::AddDataPoint(String (*Update)(void)) //Reads new data and writes data to SD
{
	String Data = "";
	Data = (*Update)(); //Run external update function
	Data = GetOnBoardVals() + Data; //Append on board readings
	LogStr(Data);
}
//ISRs
static void Margay::ButtonLog() 
{
	//ISR to respond to pressing log button and waking device from sleep and starting log
	ManualLog = true; //Set flag to manually record an additional data point
}

static void Margay::Log() 
{
	//Write global Data to SD
	LogEvent = true; //Set flag for a log event
	AwakeCount = 0; 
}

void Margay::isr0() { selfPointer->ButtonLog(); }

void Margay::isr1() { selfPointer->Log(); }

//Low Power functions
void Margay::sleepNow()         // here we put the arduino to sleep
{
	/* Now is the time to set the sleep mode. In the Atmega8 datasheet
	 * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
	 * there is a list of sleep modes which explains which clocks and
	 * wake up sources are available in which sleep mode.
	 *
	 * In the avr/sleep.h file, the call names of these sleep modes are to be found:
	 *
	 * The 5 different modes are:
	 *     SLEEP_MODE_IDLE         -the least power savings
	 *     SLEEP_MODE_ADC
	 *     SLEEP_MODE_PWR_SAVE
	 *     SLEEP_MODE_STANDBY
	 *     SLEEP_MODE_PWR_DOWN     -the most power savings
	 *
	 * For now, we want as much power savings as possible, so we
	 * choose the according
	 * sleep mode: SLEEP_MODE_PWR_DOWN
	 *
	 */  
	// MCUCR = bit (BODS) | bit (BODSE);
		// MCUCR = bit (BODS);
	wdt_disable();
	// power_adc_disable(); // ADC converter
	// // power_spi_disable(); // SPI
	// power_usart0_disable();// Serial (USART) 
	// power_timer1_disable();// Timer 1
	// power_timer2_disable();// Timer 2
	// ADCSRA = 0;
	turnOffSDcard();
	// digitalWrite(Ext3v3Ctrl, HIGH); //Turn off extenral rail
	// SPI.end(); //Turn off SPI
	// digitalWrite(SD_CS, LOW);
	// pinMode(SD_CS, INPUT); //Disconnect SD chip slect pin
	// pinMode(5, INPUT); //Set all SPI pins as inputs, will be reversed be beginning SPI again
	// pinMode(6, INPUT);
	// pinMode(7, INPUT);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

	sleep_enable();          // enables the sleep bit in the mcucr register

	// attachInterrupt(0, wakeUpNow, FALLING); // use interrupt 0 (pin 2) and run function
	                                   // wakeUpNow when pin 2 gets LOW

	sleep_mode();            // here the device is actually put to sleep!!
	                         // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
	//    sleep();              //WIRING MODE!

	sleep_disable();         // first thing after waking from sleep:
	                         // disable sleep...
	// detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
	//    ADCSRA = 1; //Turn ADC back on
	// digitalWrite(Ext3v3Ctrl, LOW); //turn external rail back on
	// digitalWrite(SD_CS, HIGH);
	// SPI.begin();
	// SD.begin(SD_CS);
	turnOnSDcard();
	// pinMode(SD_CS, OUTPUT); //Disconnect SD chip slect pin
}

void Margay::turnOffSDcard() 
{
	delay(6);
	                                   // disable SPI
	// power_spi_disable();                     // disable SPI clock
	// DDRB &= ~((1<<DDB5) | (1<<DDB7) | (1<<DDB6) | (1<<DDB4));   // set All SPI pins to INPUT
	// pinMode(SD_CD, INPUT);
	// DDRC &= ~((1<<DDC0) | (1<<DDC1));
	// pinMode(16, INPUT); //DEBUG!
	// pinMode(17, INPUT);  //DEBUG!
	// digitalWrite(16, HIGH);
	// digitalWrite(17, HIGH);
	digitalWrite(SD_CS, HIGH);
	// digitalWrite(5, LOW);
	// // Note: you must disconnect the LED on pin 13 or you’ll bleed current through the limit resistor
	// // LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); // wait 1 second before pulling the plug!
	delay(6);
	digitalWrite(Ext3v3Ctrl, HIGH);  //turn off BJT
	delay(20);
	// SPCR = SPCR & 0b11101111;
	SPCR = 0;  
	power_spi_disable(); 
	// SPI.end();
	delay(10);
	// pinMode(5, OUTPUT);
	// digitalWrite(5, LOW);
	// DDRB &= ~((1<<DDB5));
	// PORTB &= ~(1<<PORTB5); //Set port B5 (MOSI) LOW
	// DDRB &= ~((1<<DDB5) | (1<<DDB7) | (1<<DDB6) | (1<<DDB4)); 
	// PORTB |= ((1<<DDB5) | (1<<DDB7) | (1<<DDB6) | (1<<DDB4));     // set ALL SPI pins HIGH (~30k pullup)
	// digitalWrite(SD_CS, LOW);
	// pinMode(SD_CS, INPUT);
	delay(6); 
} 

void Margay::turnOnSDcard() 
{
	// pinMode(SD_CS, OUTPUT);
	// SPI.begin();
	// SD.begin(SD_CS);
	// DDRB |= ((1<<DDB5));
	// digitalWrite(SD_CS, HIGH);
	delay(6);                                            // let the card settle
	// some cards will fail on power-up unless SS is pulled up  ( &  D0/MISO as well? )
	// DDRC = DDRC | ((1<<DDC0) | (1<<DDC1));
	// DDRB = DDRB | (1<<DDB7) | (1<<DDB5) | (1<<DDB4); // set SCLK(D13), MOSI(D11) & SS(D10) as OUTPUT
	// Note: | is an OR operation so  the other pins stay as they were.                (MISO stays as INPUT) 
	// PORTB = PORTB & ~(1<<DDB7);  // disable pin 13 SCLK pull-up – leave pull-up in place on the other 3 lines
	power_spi_enable();                      // enable the SPI clock 
	SPCR=keep_SPCR;                          // enable SPI peripheral
	delay(20);
	digitalWrite(Ext3v3Ctrl, LOW); //turn on the BJT on SD ground line
	delay(10);  
}
