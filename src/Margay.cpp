/*
Margay Library
Licensed: GNU GPL v3

Written by:
Bobby Schulz
Andy Wickert
*/

#include <Margay.h>
#include <Arduino.h>


volatile bool manualLog = false; // Global for interrupt access

volatile uint8_t ExtIntPin = 255; // external interrupt pin; 255 = not set
String ext_int_header_entry;
volatile bool ExtIntTripped = false; // Global for the external interrupt
volatile uint16_t ExtInt_count = 0; // Global for the external interrupt

Margay* Margay::selfPointer;

Margay::Margay(board model_, build specs_) {
  if (model_ == 2) {
    SD_CS = 4;
    AuxLED = 20;
    RedLED = 13;
    GreenLED = 15;
    BlueLED = 14;

    VRef_Pin = 3;
    ThermSense_Pin = 1;
    BatSense_Pin = 2;

    VSwitch_Pin = 12;
    SD_CD = 1;

    Ext3v3Ctrl = 22;
    I2C_SW = 21;
    PG = 18;
    TX = 11;
    RX = 10;
    ExtInt = 11; //Legacy inclusion
    RTCInt = 2;
    LogInt = 28;

    WDHold = 23;
    BatSwitch = 19;

    BatteryDivider = 2.0;

    if (specs_ == BUILD_A) {
      NumADR_OB = 1; //Only check for clock presance
    }

    else if (specs_ == BUILD_B) {
      NumADR_OB = 2; //Tell system to search additional ADRs
      I2C_ADR_OB[1] = 0x69; //Use 0x69 on board ADC (MCP3421A1)
    }

    else if (specs_ == BUILD_C) {
      NumADR_OB = 2; //Tell system to search additional ADRs
      I2C_ADR_OB[1] = 0x6B; //Use 0x6B on board ADC (MCP3421A3)
    }
  }
  else if (model_ == 1) {
    SD_CS = 4;
    AuxLED = 20;
    RedLED = 13;
    GreenLED = 15;
    BlueLED = 14;

    VRef_Pin = 2;
    ThermSense_Pin = 1;
    BatSense_Pin = 0;

    VSwitch_Pin = 12;
    SD_CD = 1;

    Ext3v3Ctrl = 19;
    I2C_SW = 21;
    PG = 18;
    TX = 11;
    RX = 10;
    ExtIntPin = 11;
    RTCInt = 10;
    LogInt = 2;

    WDHold = 255; //Null pins
    BatSwitch = 255; //Null pins

    BatteryDivider = 2.0;

    if (specs_ == BUILD_A) {
      NumADR_OB = 1; //Only check for clock presance
    }

    else if (specs_ == BUILD_B) {
      NumADR_OB = 2; //Tell system to search additional ADRs
      I2C_ADR_OB[1] = 0x69; //Use 0x69 on board ADC (MCP3421A1)
    }

    else if (specs_ == BUILD_C) {
      NumADR_OB = 2; //Tell system to search additional ADRs
      I2C_ADR_OB[1] = 0x6B; //Use 0x6B on board ADC (MCP3421A3)
    }


    else if (specs_ == BUILD_D) {
      NumADR_OB = 2; //Tell system to search additional ADRs
      I2C_ADR_OB[1] = 0x6A; //Use 0x6A on board ADC (MCP3421A2)
    }
  }
  else {
    SD_CS = 4;
    AuxLED = 19;
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
    ExtIntPin = 11;
    RTCInt = 10;
    LogInt = 2;
    BatteryDivider = 9.0;

    if (specs_ == BUILD_A) {  //Setup sub builds
      NumADR_OB = 1; //Only check for clock presance
    }

    else if (specs_ == BUILD_B) {
      NumADR_OB = 2; //Tell system to search additional ADRs
      I2C_ADR_OB[1] = 0x69; //Use 0x69 on board ADC (MCP3421A1)
    }

    else if (specs_ == BUILD_C) {
      NumADR_OB = 2; //Tell system to search additional ADRs
      I2C_ADR_OB[1] = 0x6B; //Use 0x6B on board ADC (MCP3421A3)
    }
  }

  Model = model_; //Store model info locally
  Specs = specs_; //Store build info locally
}

void Margay::begin(uint8_t *vals, uint8_t numVals, String header_) {
  powerOB(ON);  //Turn on on-board power
  powerAux(ON); //Turn on external auxilary power
  pinMode(WDHold, OUTPUT);

  pinMode(AuxLED, OUTPUT);
  digitalWrite(AuxLED, LOW); //Turn built in LED on

  pinMode(VSwitch_Pin, OUTPUT); //Setup switch control as output

  memcpy(I2C_ADR, vals, sizeof(I2C_ADR)); //Copy array
  NumADR = numVals; //Copy length of array
  if (ExtIntPin == 255) {
    Header = header_; //Copy user defined header
  }
  else {
    Header = header_ + ext_int_header_entry;
  }

  RTC.begin(); //Initalize RTC
  RTC.clearAlarm(); //
  initADC(18);
  EnviroSense.begin(0x77); //Initalize onboard temp/pressure/RH sensor (BME280)


  ADCSRA = 0b10000111; //Confiure on board ADC for low speed, and enable

  Serial.begin(38400); //DEBUG!
  Serial.print("Lib = ");
  Serial.println(LibVersion);
  Serial.print("SN = ");
  int EEPROMLen = EEPROM.length(); //Copy value for faster access
  int val = 0; //Value to read temp EEPROM values into
  int pos = 0; //used to keep track of position in SN string
  for (int i = EEPROMLen - 8; i < EEPROMLen; i++) {  //Read out Serial Number
    val = EEPROM.read(i);  //Read SN values as individual bytes from EEPROM
    // Load upper and lower nibbles of each EEPROM byte into SN string,
    // post-incrementing the position index each time
    SN[pos++] = HexMap[(val >> 4)];
    SN[pos++] = HexMap[(val % 0x10)];
    if (i % 2 == 1 && i < EEPROMLen - 1) {
      SN[pos++] = '-';  //Place - between each SN category, post inc pos
    }
    SN[19] = '\0'; //Null terminate string
  }

  Serial.print(SN); //Print compiled string
  if (strcmp(SN, "FFFF-FFFF-FFFF-FFFF") == 0)
    Serial.println("WARNING: no serial number programmed in EEPROM");
  Serial.print("\n\n");
  Serial.println("\nInitializing...\n"); //DEBUG!
  delay(100);
  if (Serial.available()) {  //If time setting info available
    String dateTimeTemp = Serial.readString();
    Serial.println(dateTimeTemp);  //DEBUG!
    int dateTimeVals[6] = {0};
    for (int i = 0; i < 6; i++) {
      dateTimeVals[i] = dateTimeTemp.substring(2*i, 2*(i+1)).toInt();
      Serial.print(i); Serial.print("  "); //DEBUG!
      Serial.println(dateTimeVals[i]); //DEBUG!
    }
    RTC.setTime(2000 + dateTimeVals[0], dateTimeVals[1], dateTimeVals[2],
                dateTimeVals[3], dateTimeVals[4], dateTimeVals[5]);
  }

  getTime(); //Get time to pass to computer
  Serial.print("\nTimestamp = ");
  Serial.println(LogTimeDate);

  //Sets up basic initialization required for the system
  selfPointer = this;


  pinMode(RedLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  pinMode(BlueLED, OUTPUT);

  LED_Color(OFF);

  pinMode(SD_CS, OUTPUT);

  SdFile::dateTimeCallback(dateTimeSD); //Setup SD file time setting
  // Attach ISR driven by RTC interrupt; triggers data logging each interval
  attachInterrupt(digitalPinToInterrupt(RTCInt), Margay::isr1, FALLING);
  if (Model < 2) {
    // Attach ISR driven by manual log button, sets logging flag and logs data
    attachInterrupt(digitalPinToInterrupt(LogInt), Margay::isr0, FALLING);
  }
  else { //Model >= v2.0: use PCINT for log button (LogInt = D28, PA4); enable pin first
    *digitalPinToPCMSK(LogInt) |= bit(digitalPinToPCMSKbit(LogInt)); // enable
    PCIFR |= bit(digitalPinToPCICRbit(LogInt)); // clear outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(LogInt)); // enable interrupt group
  }
  pinMode(RTCInt, INPUT_PULLUP);
  pinMode(LogInt, INPUT);

  I2Ctest();
  clockTest();
  SDtest();
  batTest();
  // Only print out environmental variables if BME280 is on board
  if (Model >= MODEL_2v0) enviroStats();

  digitalWrite(AuxLED, HIGH);

  if (OBError) {
    LED_Color(RED); //On board failure
    delay(2000);
  }
  if (SensorError) {
    LED_Color(ORANGE);  //Sensor failure
    delay(2000);
  }
  if (TimeError) {
    LED_Color(CYAN); //Time set error
    delay(2000);
  }
  if (SDError) {
    LED_Color(PURPLE); //Sd card not inserted
    delay(2000);
  }
  // Battery voltage is below level where hardware functionality
  // can be guaranteed
  if (BatError) {
    for (int i = 0; i < 10; i++) {
      LED_Color(RED);
      delay(100);
      LED_Color(OFF);
      delay(100);
    }
  }

  // Battery charge % is at a concerning level; recommend replacing batteries
  if (BatWarning && !BatError) {
    for (int i = 0; i < 10; i++) {
      LED_Color(GOLD); //Sd card not inserted
      delay(100);
      LED_Color(OFF);
      delay(100);
    }
  }
  //Include battery error in test??
  if (!OBError && !SensorError && !TimeError && !SDError) {
    LED_Color(GREEN);
    delay(2000);
  }

  Serial.print("\nReady to Log...\n\n");
  NewLog = true; //Set flag to begin new log file

  if (ExtIntPin != 255) {
    pinMode(ExtIntPin, INPUT);
    digitalWrite(ExtIntPin, HIGH);
    attachInterrupt(digitalPinToInterrupt(ExtIntPin), Margay::isr2, FALLING);
  }

  LED_Color(OFF);
}

ISR (PCINT0_vect) { // handle pin change interrupt for D24-D31 (Port A) on ATmega1284p
  // NOTE: PCINT fires on both rising and falling edges. The current
  // implementation sets manualLog unconditionally. If the button is still
  // held when the logger finishes processing and re-enters sleep, the
  // rising edge on release will trigger a second log entry. Consider
  // checking pin state to fire only on the falling edge (button press):
  //   if (!(PINA & digitalPinToBitMask(28))) manualLog = true;
  manualLog = true;
}

void Margay::begin(String header_) {
  uint8_t dummy[1] = {0};
  begin(dummy, 0, header_); //Call generalized begin function
}

void Margay::I2Ctest() {
  bool initialStateExternalI2C = digitalRead(I2C_SW);

  switchExternalI2C(ON);

  int error = 0;
  bool i2cTest = true;

  Serial.print("I2C: ");
  for (int i = 0; i < NumADR; i++) {
    Wire.beginTransmission(I2C_ADR[i]);
    error = Wire.endTransmission();
    if (error != 0) {
      if (i2cTest) Serial.println(" Fail");
      Serial.print("   Fail At: ");
      Serial.println(I2C_ADR[i], HEX);
      i2cTest = false;
      SensorError = true;
    }
  }

  //Switch to connect to onboard I2C!
  switchExternalI2C(OFF);

  for (int i = 0; i < NumADR_OB; i++) {
    Wire.beginTransmission(I2C_ADR_OB[i]);
    error = Wire.endTransmission();
    if (error != 0) {
      if (i2cTest) Serial.println(" Fail");
      Serial.print("   Fail At: ");
      Serial.println(I2C_ADR_OB[i], HEX);
      i2cTest = false;
      OBError = true;
    }
  }

  if (i2cTest) Serial.println("PASS");

  // make sure I2C Bus is returned to initial state
  farmGateI2C(initialStateExternalI2C);
}


void Margay::SDtest() {
  bool sdErrorTemp = false;

  // SD_CD is pulled up: HIGH=1 if not present
  // SD card being inserted closes a switch to pull it LOW
  pinMode(SD_CD, INPUT);
  bool cardNotPresent = digitalRead(SD_CD);

  Serial.print("SD: ");
  delay(5); //DEBUG!
  if (cardNotPresent) {
    Serial.println(F(" NO CARD"));
    sdErrorTemp = true;
    SDError = true; //Card not inserted
  }
  else if (!SD.begin(SD_CS)) {
    OBError = true;
    sdErrorTemp = true;
  }

  // If card is present, do the following:
  if (!cardNotPresent) {
    SD.mkdir("NW");  //Create NW folder (if not already present)
    SD.chdir("/NW"); //Move file pointer into NW folder (at root level)
    SD.mkdir(SN); //Make directory with serial number as name
    SD.chdir(SN); //Move into this directory
    //Change directory to SN# named dir
    SD.mkdir("Logs"); //Use???
    String fileNameTest = "HWTest";
    (fileNameTest + ".txt").toCharArray(FileNameTestC, 11);
    SD.remove(FileNameTestC); //Remove any previous files

    // Seed with a random process to ensure randomness
    randomSeed(analogRead(A7));
    // Generate a random number between 0 and 30557
    // (the number of words in Hamlet)
    int randVal = random(30557);
    char randDigits[6] = {0};
    // Convert randVal into a series of digits
    sprintf(randDigits, "%d", randVal);
    // Find the length of the digit string
    int randLength = (int)((ceil(log10(randVal))+1)*sizeof(char));
    File dataWrite = SD.open(FileNameTestC, FILE_WRITE);
    if (dataWrite) {
      dataWrite.println(randVal);
      dataWrite.println("\nHe was a man. Take him for all in all.");
      dataWrite.println("I shall not look upon his like again.");
      dataWrite.println("-Hamlet, Act 1, Scene 2");
    }
    dataWrite.close();
    char testDigits[6] = {0};
    File dataRead = SD.open(FileNameTestC, FILE_READ);
    if (dataRead) {
      dataRead.read(testDigits, randLength);
      for (int i = 0; i < randLength - 1; i++){ //Test random value string
        if (testDigits[i] != randDigits[i]) {
          sdErrorTemp = true;
          OBError = true;
        }
      }
    }
    dataRead.close();

    keep_SPCR=SPCR;
  }

  // If card is inserted and still does not connect properly, throw error
  if (sdErrorTemp && !cardNotPresent) Serial.println("FAIL");
  // If card is inserted AND connects properly, return success
  else if (!sdErrorTemp && !cardNotPresent) Serial.println("PASS");
}

void Margay::clockTest() {
  int error = 1;
  uint8_t testSeconds = 0;
  bool oscStop = false;

  Serial.print("Clock: ");
  Wire.beginTransmission(I2C_ADR_OB[0]);
  Wire.write(0xFF);
  error = Wire.endTransmission();

  if (error == 0) {
    getTime(); //FIX!
    testSeconds = RTC.getValue(5);
    delay(1100);
    if (RTC.getValue(5) == testSeconds) {
      OBError = true; //If clock is not incrementing
      oscStop = true; //Oscilator not running
    }
  }

  unsigned int yearNow = RTC.getValue(0);

  // DS3231 powers on at Jan 1, 2000 (year register = 0). If the year
  // is still 0, the clock has not been set — timestamps will read as
  // 2000, which is an obviously wrong but identifiable sentinel value.
  if (yearNow == 00) {
    TimeError = true;
    Serial.println(" PASS, BAD TIME");
  }

  if (error != 0) {
    Serial.println(" FAIL");
    OBError = true;
  }

  else if (error == 0 && oscStop == false && TimeError == false) {
    Serial.println(" PASS");
  }
}

void Margay::batTest() {
  float batVoltage = getBatVoltage();
  float batPercentage = getBatPercentage();
  // Set error flag if below min voltage
  if (batVoltage < BatVoltageError) BatError = true;
  // Set warning flag if below set percentage
  if (batPercentage < BatPercentageWarning) BatWarning = true;
  Serial.print("Bat = ");
  Serial.print(batVoltage);
  Serial.print("V\t");
  Serial.print(batPercentage);
  Serial.println("%");
}

void Margay::initADC(uint8_t desiredResolution) {
  // Serial.print("ADC should be on"); // DEBUG
  adc.Begin(I2C_ADR_OB[1]); //Initalize external ADC
  adc.SetResolution(desiredResolution);
}

void Margay::powerTest() {
  int error = 0;

  digitalWrite(Ext3v3Ctrl, HIGH); //Turn off power to outputs

  Serial.print("Power: ");
  Wire.beginTransmission(I2C_ADR[1]);
  error = Wire.endTransmission();
  if (error == 0) Serial.println(" FAIL");

  if (error != 0) Serial.println(" PASS");

  digitalWrite(Ext3v3Ctrl, LOW); //Turn power back on
}

void Margay::enviroStats() {
  Serial.print("Temp = ");
  Serial.print(EnviroSense.GetTemperature());
  Serial.println("C");
  Serial.print("Pressure = ");
  Serial.print(EnviroSense.GetPressure());
  Serial.println(" mBar");
  Serial.print("RH = ");
  Serial.print(EnviroSense.GetHumidity());
  Serial.println("%");
}

void Margay::initLogFile() {
  SD.chdir("/NW");  //Move into northern widget folder from root
  SD.chdir(SN);  //Move into specific numbered sub folder
  SD.chdir("Logs"); //Move into the logs sub-folder
  //Perform same search, but do so inside of "SD:NW/sn/Logs"
  char numCharArray[6];
  String fileName = "Log";
  int fileNum = 1;
  sprintf(numCharArray, "%05d", fileNum);
  (fileName + String(numCharArray) + ".txt").toCharArray(FileNameC, 13);
  while (SD.exists(FileNameC)) {
    fileNum += 1;
    sprintf(numCharArray, "%05d", fileNum);
    (fileName + String(numCharArray) + ".txt").toCharArray(FileNameC, 13);
  }
  Serial.print("FileNameC: ");
  Serial.println(FileNameC);
  // Make string of onboard characteristics as first line of data
  String initData = "Lib = " + String(LibVersion) + " SN = " + String(SN);
  logStr(initData);
  // Log concatenated header (old loggers lack BME280)
  if (Model < MODEL_2v0)
    logStr("Time [UTC], Temp OB [C], Temp RTC [C], Bat [V], " + Header);
  else  // new loggers include pressure and RH from BME280
    logStr("Time [UTC], PresOB [mBar], RH_OB [%], TempOB [C], "
           "Temp RTC [C], Bat [V], " + Header);
}

int Margay::logStr(String val) {
  Serial.println(val); //Echo to serial monitor
  SD.chdir("/NW");  //Move into northern widget folder from root
  SD.chdir(SN);  //Move into specific numbered sub folder
  SD.chdir("Logs"); //Move into the logs sub-folder
  File DataFile = SD.open(FileNameC, FILE_WRITE);

  // if the file is available, write to it:
  if (DataFile) {
    DataFile.println(val);
    DataFile.close();
    return 0;
  }
  // if the file isn't open, pop up an error:
  else {
    DataFile.close();
    return -1;
  }
}

void Margay::LED_Color(unsigned long val) { //Set color of onboard led
  int red = 0; //red led color
  int green = 0;  //green led color
  int blue = 0;  //blue led color
  int lum = 0;  //Luminosity

  //Parse all values from single val
  blue = val & 0xFF;
  green = (val >> 8) & 0xFF;
  red = (val >> 16) & 0xFF;
  lum = (val >> 24) & 0xFF;
  //  lum = 255 - lum; //Invert since LEDs are open drain

  analogWrite(RedLED, 255 - (red * lum)/0xFF);
  analogWrite(GreenLED, 255 - (green * lum)/0xFF);
  analogWrite(BlueLED, 255 - (blue * lum)/0xFF);
}

void Margay::getTime() {
  //Update global time string
  LogTimeDate = RTC.getTime(0);
}

float Margay::getTemp(temp_source sensor) {
  float vcc = 3.3;
  // Get temp from on board thermistor
  if (sensor == thermistor_temp_sensor) {
    float adcVoltage = float(analogRead(ThermSense_Pin))*(vcc/1023.0);
    float tempData = tempConvert(adcVoltage, vcc, 10000.0, A, B, C, D, 10000.0);
    tempData = tempData - 273.15;
    return tempData;
  }
  // Get Temp from RTC
  else if (sensor == RTC_temp_sensor) {
    float rtcTemp = RTC.getTemp();
    return rtcTemp;
  }
  else {
    // Obvious temperature error value that no sensor would give
    return -1234; 
  }
}

float Margay::getBatVoltage() {
  // Maybe not necessary: seems to be set this way anyway
  // Enable ADC, set clock divider to max to deal with high impedance input
  ADCSRA = 0b10000111;
  delay(10); //Alow for >1 clock cycle to set values

  float vAux = 3.3; // Voltage reference for ATMega1284p ADC
  float batADC10bit = analogRead(BatSense_Pin); //Get (divided) battery ADC val
  //VRef is having issues: often approx 0.9 <-- This was from the hardware component; fixed now
  // Therefore, instead we will just use the 3V3 regulator as our basis
  // Find compensation value with VRef due to larger uncertainty with vcc
  float comp = (1.8/3.3)*1023./analogRead(VRef_Pin);
  // Override comp calculation since many v0.0 models do not have ref equipped
  if (Model == 0) comp = 1.0;
  // Should divide by 1023. instead of 1024: 0-1023
  //batVoltage = batVoltage*BatteryDivider*comp*(vcc/1024.0);
  //  Compensate for voltage divider and ref voltage error
  float batVoltage = batADC10bit/1023. * vAux * BatteryDivider;
  return batVoltage;
}

float Margay::getBatPercentage() {
  if (NCells == 0) {
    Serial.println(F("getBatPercentage: NCells must be > 0"));
    return -1;
  }
  // NOTE: Fit developed for Duracell AA, should work well for most alkalines,
  // but no guarantee given on accuracy
  // From 30% to 100% capacity, should be accurate to within 1%
  // (for data taken at 25C)
  float batA = -1.9809;
  float batB = 6.2931;
  float batC = -4.0063;
  float cellVoltage = getBatVoltage()/NCells; //Divide to get per-cell voltage
  // Return percentage of remaining battery energy
  float percentage = ((batA*pow(cellVoltage, 2) + batB*cellVoltage + batC)*2 - 1)*100.0;
  if (percentage < 0) return 0;  //Do not allow return of non-sensical values
  // Is this appropriate? Float voltage could be higher than specified
  // and still be correct
  if (percentage > 100) return 100;
  return percentage;
}

String Margay::getOnBoardVals() {
  //Get onboard temp, RTC temp, and battery voltage, referance voltage
  // float VRef = analogRead(VRef_Pin);
  float vcc = 3.3; //(1.8/VRef)*3.3; //Compensate for vcc using VRef
  // Serial.println(vcc); //DEBUG!
  float tempData = 0; //FIX!!! Dumb!

  if (Model < MODEL_2v0) {  //For older thermistor models
    float val = float(analogRead(ThermSense_Pin));
    // Find compensation value with VRef due to vcc error
    float comp = (1.8/3.3)*1023.0/analogRead(VRef_Pin);
    // Override comp calculation since many v0.0 models do not have ref equipped
    if (Model == 0) comp = 1.0;
    val = val*comp*(vcc/1023.0); //Compensate for ref voltage error
    //  float Vout = vcc - val;
    //  Serial.println(val); //DEBUG!
    //  Serial.println(Vout);  //DEBUG!
    tempData = tempConvert(val, vcc*comp, 10000.0, A, B, C, D, 10000.0);
    tempData = tempData - 273.15; //Get temp from on board thermistor
  }

  // delay(10);
  // Get battery voltage, including voltage divider in math
  float batVoltage = getBatVoltage();

  // Temp[3] = Clock.getTemperature(); //Get tempreture from RTC //FIX!
  float rtcTemp = RTC.getTemp();  //Get Temp from RTC
  getTime(); //FIX!
  if (Model < MODEL_2v0)
    return LogTimeDate + "," + String(tempData) + ","
           + String(rtcTemp) + "," + String(batVoltage) + ",";
  else
    return LogTimeDate + "," + String(EnviroSense.GetString())
           + String(rtcTemp) + "," + String(batVoltage) + ",";
}

float Margay::tempConvert(float V, float vcc, float R,
    float A, float B, float C, float D, float R25) {
  //  Serial.print("R = "); //DEBUG!
  //  Serial.println(R); //DEBUG!
  float rt = ((vcc/V)*R) - R;
  //  Serial.print("rt = "); //DEBUG!
  //  Serial.println(rt); //DEBUG!
  float logRt = log(rt/R25);
  //  Serial.print("logRt = "); //DEBUG!
  //  Serial.println(logRt); //DEBUG!
  float t = 1.0/(A + B*logRt + C*pow(logRt, 2.0) + D*pow(logRt, 3.0));
  return t;
}

void Margay::blinkGood() {
  // Peppy blinky pattern to show that the logger has successfully initialized
  digitalWrite(BlueLED,LOW);
  delay(651);
  digitalWrite(BlueLED,HIGH);
  delay(300);
  digitalWrite(BlueLED,LOW);
  delay(100);
  digitalWrite(BlueLED,HIGH);
  delay(200);
  digitalWrite(BlueLED,LOW);
  delay(100);
  digitalWrite(BlueLED,HIGH);
}

float Margay::getVoltage() {  //Get voltage from Ax pin
  // Voltage reads from the on-board ADC, but to read the Ax pin at the
  // same time as external sensors, need access to the ADC. However, we
  // do not want to change the state of the I2C bus communication by
  // taking a voltage reading. So we have logic here to make the switch.

  // First check whether external I2C connections are on by testing pin
  // I2C_SW (HIGH is on). When the external I2C connection is on, the
  // internal I2C connection is cut off.
  bool initialStateExternalI2C = digitalRead(I2C_SW);

  // initialize a variable to hold the voltage reading.
  float val = 0;

  switchExternalI2C(OFF);
  initADC(18); // initialize ADC
  val = adc.GetVoltage();

  // make sure I2C Bus is returned to initial state
  farmGateI2C(initialStateExternalI2C);

  return val;
}

// Pass in function which returns string of data
void Margay::run(String (*update)(void), unsigned long logInterval) {
  // Print note that that logging has started
  // Serial.println("Log Started!"); //DEBUG!
  // Serial.println(millis()); //DEBUG!
  if (NewLog) {
    // LogEvent = true;
    RTC.setAlarm(logInterval);
    initLogFile(); //Start a new file each time log button is pressed
    //Add inital data point
    addDataPoint(update);
    NewLog = false;  //Clear flag once log is started
    blinkGood();  //Alert user to start of log
    resetWDT(); //Clear alarm
  }

  if (LogEvent) {
    // Serial.println("Log Event!"); //DEBUG!
    // RTC.setAlarm(logInterval);  //Set/reset alarm //DEBUG!
    addDataPoint(update); //Write values to SD
    LogEvent = false; //Clear log flag
    RTC.setAlarm(logInterval);  //Set/reset alarm
    resetWDT(); //Clear alarm
  }

  // Write data to SD card without interrupting existing timing cycle
  if (manualLog) {
    // Serial.println("Click!"); //DEBUG!
    addDataPoint(update); //write values to SD
    manualLog = false; //Clear log flag
    resetWDT(); //Clear alarm
  }

  if (ExtIntTripped) {  // Defaults to just counter for now
    // Serial.println("TIP!"); //DEBUG!
    ExtInt_count ++;
    ExtIntTripped = false; // Clear interrupt flag flag
    resetWDT(); //Clear alarm
    delay(150); //Hard-code for now; tipping bucket "debounce"
    attachInterrupt(digitalPinToInterrupt(ExtIntPin), Margay::isr2, FALLING);
  }

  if (!digitalRead(RTCInt)) {  //Catch alarm if not reset properly
    Serial.println("Reset Alarm"); //DEBUG!
    RTC.setAlarm(logInterval); //Turn alarm back on
  }

  AwakeCount++;

  // @bschulz1701: AwakeCount was designed to give ~5 run() iterations after
  // an RTC wake before returning to sleep, reset to 0 by the RTC ISR
  // (writeDataToSD). Since addDataPoint() blocks within a single run() call,
  // the 5-count may be unnecessary. Consider simplifying or removing.
  if (AwakeCount > 5) {
    sleepNow();
  }
  delay(1);
}

// Send a pulse to "feed" the watchdog timer
void Margay::resetWDT() {
  digitalWrite(WDHold, HIGH); //Set DONE pin high
  delayMicroseconds(5); //Wait a short pulse
  digitalWrite(WDHold, LOW);
}

void Margay::switchExternalI2C(bool desiredState) {
  /*
  Must be ON to read off-board sensors, OFF to read on-board sensors and RTC
  Serves to isolate these s.t. I2C addresses may not clash.
  */
  pinMode(I2C_SW, OUTPUT);

  if ( desiredState == ON ) {
    digitalWrite(I2C_SW, HIGH);
    externalI2COn = digitalRead(I2C_SW);
  }
  else {
    digitalWrite(I2C_SW, LOW);
    externalI2COn = digitalRead(I2C_SW);
  }
  delay(1); // Any time needed to switch states; may not be necessary
}

void Margay::farmGateI2C(bool initStateI2C) {
  // This closes the farm gate at the end of a function that uses the
  // external I2C bus. It works by checking if the current state of I2C
  // comms (class variable) matches the starting state (argument to this
  // function). If they do not match (XOR), and it was on at the start,
  // then turn on. Otherwise, turn off.

  if ((initStateI2C == !externalI2COn) && initStateI2C) {
    switchExternalI2C(ON);
  }

  else {
    switchExternalI2C(OFF);
  }
}

// Reads new data and writes data to SD
void Margay::addDataPoint(String (*update)(void)) {
  String data = "";
  //Re-initialize BME280  //FIX??
  if (Model >= MODEL_2v0) EnviroSense.begin(0x77);
  // Serial.println("Called Update"); //DEBUG!

  bool initialStateExternalI2C = digitalRead(I2C_SW);

  switchExternalI2C(ON);
  data = (*update)(); //Run external update function

  // make sure I2C Bus is returned to initial state
  farmGateI2C(initialStateExternalI2C);

  _addDataPoint(data);

}

void Margay::_addDataPoint(String data) {
  // Serial.println("Request OB vals"); //DEBUG!
  // Briefly flash an LED to show that data are being logged
  // without needing to waste extra time/power with a delay.
  // This step should always take the same amount of time
  // unless there is a significant library or xtal change
  pinMode(BlueLED, OUTPUT);
  digitalWrite(BlueLED, LOW); //ON
  data = getOnBoardVals() + data; //Prepend on board readings
  digitalWrite(BlueLED, HIGH); //OFF
  // Serial.println("Got OB vals");  //DEBUG!
  logStr(data);
  // Serial.println("Logged Data"); //DEBUG!
}

//ISRs

void Margay::buttonLog() {
  // ISR to respond to pressing log button and waking device from sleep
  // and starting log
  manualLog = true; //Set flag to manually record an additional data point
}

void Margay::extIntCounter() {
  // ISR for an external event waking the logger
  detachInterrupt(digitalPinToInterrupt(ExtIntPin));
  // Set flag to just increment the counter and return to sleep
  ExtIntTripped = true;
}

void Margay::writeDataToSD() {
  //Write global data to SD
  LogEvent = true; //Set flag for a log event
  AwakeCount = 0;
}

// ExtInt functions
void Margay::setExtInt(uint8_t n, String header_entry) {
  ExtIntPin = n;
  ext_int_header_entry = header_entry;
}

uint16_t Margay::getExtIntCount(bool reset0) {
  uint16_t out = ExtInt_count;
  if (reset0) {
    resetExtIntCount(0);
  }
  return out;
}

void Margay::resetExtIntCount(uint16_t start) {
  ExtInt_count = start;
}

void Margay::powerAux(bool state) {
  pinMode(Ext3v3Ctrl, OUTPUT); //Setup outputs for robustness
  if (state) powerOB(ON); //Turn on on-board power if required
  if (Model >= MODEL_2v0) {  //use positive logic for Model v2.0 and newer
    digitalWrite(Ext3v3Ctrl, state); //Switch 3v3 Aux power
  }
  else digitalWrite(Ext3v3Ctrl, !state); //Switch 3v3 Aux power
}

void Margay::powerOB(bool state) {
  pinMode(BatSwitch, OUTPUT);
  digitalWrite(BatSwitch, state); //Set bat switch for onboard 3v3/main power
}

void Margay::dateTimeSD(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(selfPointer->RTC.getValue(0) + 2000,
                   selfPointer->RTC.getValue(1),
                   selfPointer->RTC.getValue(2));

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(selfPointer->RTC.getValue(3),
                   selfPointer->RTC.getValue(4),
                   selfPointer->RTC.getValue(5));
}

void Margay::isr0() { selfPointer->buttonLog(); }
void Margay::isr1() { selfPointer->writeDataToSD(); }
void Margay::isr2() { selfPointer->extIntCounter(); }

//Low Power functions
void Margay::sleepNow() {         // here we put the arduino to sleep
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and
   * wake up sources are available in which sleep mode.
   *
   * In the avr/sleep.h file, the call names of these sleep modes
   * are to be found:
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
  turnOffSDcard();
  digitalWrite(VSwitch_Pin, LOW); //DEBUG!
  keep_ADCSRA = ADCSRA;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  cbi(ADCSRA,ADEN);
  sleep_enable();
  sleep_bod_disable();
  sei();

  sleep_cpu();
  sleep_disable();
  turnOnSDcard();
  ADCSRA = 135; //DEBUG!

}

void Margay::turnOffSDcard() {
  delay(6);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  delay(6);
  powerAux(OFF); //turn off external 3v3 rail
  powerOB(OFF); //Turn off battery connection to sense divider
  delay(1);
  digitalWrite(SD_CS, LOW);
  delay(20);
  SPCR = 0;
  power_spi_disable();
  delay(10);
  delay(6);
}

void Margay::turnOnSDcard() {
  powerOB(ON); //Turn on battery connection to sense divider
  powerAux(ON); //turn on external 3v3 rail
  delay(6);                   // let the card settle
  power_spi_enable();         // enable the SPI clock
  SPCR = keep_SPCR;           // enable SPI peripheral
  delay(10);
  SD.begin(SD_CS, SD_SCK_MHZ(8));
}
