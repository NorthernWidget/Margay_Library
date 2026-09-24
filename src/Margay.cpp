/*
Margay Library
Licensed: GNU GPL v3

Written by:
Bobby Schulz
Andy Wickert
*/

#include <Margay.h>
#include <Arduino.h>

// CRC-8/SMBUS (polynomial 0x07, init 0x00) over Page 0 bytes 0x00-0x1D, as
// NW-Provision writes it (NW-Device-Specification Page 0 Block 3).
static uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}


Margay::Margay(board model_, build specs_) {
  if (model_ == 2 || model_ == 3) {
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
      NumADR_OB = 1; //Only check for clock presence
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
      NumADR_OB = 1; //Only check for clock presence
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
      NumADR_OB = 1; //Only check for clock presence
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

bool Margay::begin(uint8_t *vals, uint8_t numVals, String header_) {
  powerOB(ON);  //Turn on on-board power
  powerAux(ON); //Turn on external auxiliary power
  if (WDHold != 255) pinMode(WDHold, OUTPUT);

  pinMode(AuxLED, OUTPUT);
  digitalWrite(AuxLED, LOW); //Turn built in LED on

  pinMode(VSwitch_Pin, OUTPUT); //Setup switch control as output

  acceptAddresses(vals, numVals, header_); //The sketch's sensor addresses and header

  RTC.begin(); //Initialize RTC
  RTC.clearAlarm(); //
  if (NumADR_OB > 1) initADC(18); // Only BUILD_B/C/D have an on-board ADC
  if (Model >= MODEL_2v0 && !bme280.begin(0x77)) { //Initialize onboard temp/pressure/RH sensor (BME280)
    Serial.println("BME280 init: FAIL");
    OnBoardError = true;
    BMEError = true;
  }


  ADCSRA = 0b10000111; //Configure on board ADC for low speed, and enable

  Serial.begin(38400); //DEBUG!
  Serial.print("Lib = ");
  Serial.println(LibVersion);
  Serial.print("Model = ");
  Serial.print(Model);
  Serial.print("  Build = ");
  Serial.println(Specs);
  bool schema1 = readIdentity(); //Serial number and hardware version from Page 0 (Schema 1), else the Schema 0 bytes
  if (!schema1) HWVersion = String(Model); //Schema 0: the model number the sketch declared
  if (schema1 && !Pages.page1Blank()) { //Page 1: this board's calibration, written by NW-Provision; the constants otherwise
    BatteryDivider = Pages.get16(0x20) / 1000.0;
    A = Pages.getFloat(0x22); B = Pages.getFloat(0x26); C = Pages.getFloat(0x2A); D = Pages.getFloat(0x2E);
    BatVoltageError = Pages.get16(0x32) / 100.0;
    BatPercentageWarning = Pages.page[0x34];
    Serial.println("Calibration from Page 1");
  }
  serialTimeSet(); //A YYMMDDHHMMSS string waiting on Serial sets the clock; then the timestamp
  attachLoggerInterrupts(Model >= 2); //LED pins, SD chip select, file times, the alarm and the button (PCINT from v2.0)

  I2Ctest();
  clockTest();
  SDtest();
  batTest();
  powerTest();
  // Only print out environmental variables if BME280 is on board
  if (Model >= MODEL_2v0) bme280Readings();

  ledReport(); //The self-test results on the RGB LED, then "Ready to Log"
  //The logger's own report at boot, for its first status row: the first fault the
  //self-tests found, else LoggingStarted (unit, kind 16).
  if (SDCardMissing) Pages.latchFault(0x01);
  else if (SDTestFailed) Pages.latchFault(0x05);
  else if (ClockError) Pages.latchFault(0x21);
  else if (BMEError) Pages.latchFault(0x41);
  else if (SensorError) Pages.latchFault(0x61);
  else if (BatError) Pages.latchFault(0x84);
  Pages.latchNotice(0xF0);
  BootReport = Pages.report();
  Pages.acknowledge();
  NewLog = true; //Set flag to begin new log file

  attachExtInt(); //The external-interrupt counter, if setExtInt() named a pin

  LED_Color(OFF);
  return !(OnBoardError || SensorError || TimeError || SDCardMissing); //Okapi's convention: true = nothing wrong
}

void Margay::batTest() {
  float batVoltage = getBatVoltage();
  float batPercentage = getBatPercentage();
  // Set error flag if below min voltage
  if (batVoltage < BatVoltageError) BatError = true;
  // Set warning flag if below set percentage
  if (batPercentage < BatPercentageWarning) { BatWarning = true; Pages.latchNotice(0x90); } //BatteryWarning
  Serial.print("Bat = ");
  Serial.print(batVoltage);
  Serial.print("V\t");
  Serial.print(batPercentage);
  Serial.println("%");
}

void Margay::initADC(uint8_t desiredResolution) {
  // Serial.print("ADC should be on"); // DEBUG
  adc.begin(I2C_ADR_OB[1]); //Initialize external ADC
  adc.setResolution(desiredResolution);
}

void Margay::powerTest() {
  // BME280 at 0x77 is on the AUX rail on Model >= 2v0; skip on older boards
  if (Model < MODEL_2v0) {
    Serial.println(F("Power: SKIP (not supported on this board)"));
    return;
  }

  Serial.print("Power: ");

  bool initialStateExternalI2C = digitalRead(I2C_SW);
  switchExternalI2C(OFF); // BME280 is on the internal I2C bus

  powerAux(OFF); // cut AUX rail
  delay(10);     // allow capacitors to discharge

  Wire.beginTransmission(0x77);
  int error = Wire.endTransmission();

  powerAux(ON);
  // Adafruit_BME280::begin() issues a soft-reset then delays >=300ms for
  // calibration — the sensor is fully ready when begin() returns, so no
  // additional settling delay is needed before bme280Readings() is called.
  bme280.begin(0x77);
  farmGateI2C(initialStateExternalI2C); // restore I2C bus to its prior state

  if (error == 0) {
    Serial.println("FAIL"); // BME280 still responded — AUX rail not cut
    OnBoardError = true;
  } else {
    Serial.println("PASS");
  }
}

void Margay::bme280Readings() {
  Serial.print("Temp = ");
  Serial.print(bme280.getTemperature());
  Serial.println("C");
  Serial.print("Pressure = ");
  Serial.print(bme280.getPressure());
  Serial.println(" mBar");
  Serial.print("RH = ");
  Serial.print(bme280.getHumidity());
  Serial.println("%");
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
  delay(10); //Allow for >1 clock cycle to set values

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

// The data file's header row: the on-board columns (old loggers lack the
// BME280), then the sketch's Header, then Note. Note is always the last
// column and carries no comma after it: every sensor ends its fields with a
// comma for the next, so this ends the row.
String Margay::dataHeader() {
  // Note is always the last column and carries no comma after it: every
  // sensor ends its fields with a comma for the next, so this ends the row.
  if (Model < MODEL_2v0)
    return "Time [UTC], Temp OB [C], Temp RTC [C], Bat [V], " + Header + "Note";
  else  // new loggers include pressure and RH from BME280
    return "Time [UTC], PresOB [mBar], RH_OB [%], TempOB [C], "
           "Temp RTC [C], Bat [V], " + Header + "Note";
}

String Margay::getOnBoardVals() {
  // Get onboard temp, RTC temp, and battery voltage, reference voltage
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

  // Temp[3] = Clock.getTemperature(); //Get temperature from RTC //FIX!
  float rtcTemp = RTC.getTemp();  //Get Temp from RTC
  getTime(); //FIX!
  if (Model < MODEL_2v0)
    return LogTimeDate + "," + String(tempData) + ","
           + String(rtcTemp) + "," + String(batVoltage) + ",";
  else
    return LogTimeDate + "," + String(bme280.getString())
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
  if (NumADR_OB > 1) { // Only BUILD_B/C/D have an on-board ADC
    initADC(18);
    val = adc.getVoltage();
  }

  // make sure I2C Bus is returned to initial state
  farmGateI2C(initialStateExternalI2C);

  return val;
}

// Reads new data and writes data to SD
void Margay::addDataPoint(String (*update)(void)) {
  String data = "";
  //Re-initialize BME280  //FIX??
  if (Model >= MODEL_2v0) bme280.begin(0x77);
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
  data = getOnBoardVals() + data + Note; //Prepend on board readings; Note column last
  Note = ""; //One row's worth of notes
  digitalWrite(BlueLED, HIGH); //OFF
  // Serial.println("Got OB vals");  //DEBUG!
  if (logStr(data) != 0) Pages.latchNotice(0xF2); //RowNotWritten
  // Serial.println("Logged Data"); //DEBUG!
  fillPages(); //Margay's reading of itself: Page 2, Page 3, Block 0
  reportRows(); //The status file: a row for the logger and every watched sensor with something to report
}

uint8_t Margay::chipFaults() {
  uint8_t f = 0;
  if (SDCardMissing || SDTestFailed) f |= 0x01;
  if (ClockError) f |= 0x02;
  if (BMEError) f |= 0x04;
  if (SensorError) f |= 0x08;
  if (BatError) f |= 0x10;
  return f;
}

void Margay::fillPages() {
  Pages.beginReading();
  float v = getBatVoltage();
  Pages.put8(0x48, (uint8_t)constrain(getBatPercentage(), 0, 100));
  Pages.put16(0x49, (uint16_t)(v * 100.0 + 0.5));
  Pages.put16(0x4B, (uint16_t)(int16_t)(getTemp(thermistor_temp_sensor) * 100.0)); //0 on models without the thermistor path
  if (Model >= MODEL_2v0 && !BMEError) {
    Pages.put16(0x50, (uint16_t)(int16_t)(bme280.getTemperature() * 100.0));
    Pages.put16(0x52, (uint16_t)(bme280.getHumidity() * 100.0));
    Pages.put32(0x54, (uint32_t)(bme280.getPressure() * 100.0));
  }
  Pages.put32(0x58, clockUnix()); //Clock: Unix seconds
  Pages.put16(0x5C, (uint16_t)(int16_t)(RTC.getTemp() * 100.0));
  Pages.put16(0x60, FileNum);
  Pages.put32(0x62, LogInterval);
  Pages.put16(0x66, getExtIntCount(false));
  Pages.endReading(chipFaults());
}

static const char* const margayChips[] = {"SDCard", "Clock", "BME280", "SensorBus", "Battery"};
static const char* const margayWords[] = {"LoggingStarted", "NewLogFile", "RowNotWritten"};   //unit kinds 16-18
static const char* const margayChipWords[] = {"BatteryWarning", "ClockSet"};   //kind 16 on Battery (0x90) and on Clock (0x30): one word each

size_t Margay::printStatus(Print& out, bool boot) {
  const NW_Report& r = boot ? BootReport : Pages.report();
  //Kind 16 means a different thing on the unit, the battery and the clock: choose the word table by chip
  const char* const* words = margayWords; uint8_t n = 3;
  if (r.chip() == 4) { words = margayChipWords; n = 1; }
  else if (r.chip() == 1) { words = margayChipWords + 1; n = 1; }
  return Pages.printSnapshot(out, margayChips, 5, LibVersion.c_str(), &r, words, n, MARGAY_LIBRARY_COMMIT, "", SKETCH_COMMIT); //A logger: its library is its firmware; the sketch stands where a library would
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
  if (BatSwitch == 255) return; // No battery switch on this board model
  pinMode(BatSwitch, OUTPUT);
  digitalWrite(BatSwitch, state); //Set bat switch for onboard 3v3/main power
}

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
  // NOTE: these are SCL (D16/PC0), SDA (D17/PC1), RX0 (D8/PD0), TX0 (D9/PD1)
  // — not SD card pins. They are released here to prevent current leakage
  // during sleep, but logically belong in sleepNow(). Deferred: moving them
  // changes the order of operations relative to power cutoff and SPCR = 0.
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
