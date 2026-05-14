/*
Margay Library
Licensed: GNU GPL v3

Written by:
Bobby Schulz
Andy Wickert
*/


#ifndef MARGAY_h
#define MARGAY_h

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <EEPROM.h>
#include "DS3231_Logger.h"
#include "MCP3421.h"
#include "SdFat.h"
#include <BME.h>


/// @defgroup colors LED color constants
/// Packed 32-bit LED color values. Format: 0xLLRRGGBB where LL = luminosity,
/// RR = red, GG = green, BB = blue. Pass to LED_Color().
/// @{
#define RED         0xFFFF0000L
#define GREEN       0xFF00FF00L
#define BLUE        0xFF0000FFL
#define MAROON      0xFF800000L
#define GOLD        0xFFFFD700L
#define ORANGE      0xFFFFA500L
#define PURPLE      0xFF800080L
#define CYAN        0xFF00FFFFL
#define BLACK_ALERT 0x802019FFL ///< Deep blue-violet; used internally for error states.
/// @}

#define ON  1
#define OFF 0

//Define CBI macro
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

/**
 * @brief Hardware model version of the Margay data logger.
 * @details Pass to the Margay constructor to select the correct pin map and
 * hardware configuration. MODEL_2v0, MODEL_2v1, and MODEL_2v2 all map to
 * the same pin configuration (value 2).
 */
enum board
{
    MODEL_0v0 = 0, ///< Margay v0.0 (prototype)
    MODEL_1v0 = 1, ///< Margay v1.0
    MODEL_2v0 = 2, ///< Margay v2.0
    MODEL_2v1 = 2, ///< Margay v2.1 (same pin map as v2.0)
    MODEL_2v2 = 2, ///< Margay v2.2 (same pin map as v2.0)
    MODEL_3v0 = 3  ///< Margay v3.0
};

/**
 * @brief Build variant of the Margay data logger.
 * @details Selects the I2C address of the on-board MCP3421 ADC.
 * BUILD_A has no on-board ADC. BUILD_B uses address 0x69 (MCP3421A1),
 * BUILD_C uses 0x6B (MCP3421A3), BUILD_D uses 0x6A (MCP3421A2).
 */
enum build
{
    BUILD_A = 0, ///< No on-board ADC
    BUILD_B = 1, ///< On-board ADC at 0x69 (MCP3421A1)
    BUILD_C = 2, ///< On-board ADC at 0x6B (MCP3421A3)
    BUILD_D = 3  ///< On-board ADC at 0x6A (MCP3421A2); MODEL_1v0 only
};

/**
 * @brief Temperature sensor selection for getTemp().
 */
enum temp_source
{
    thermistor_temp_sensor = 0, ///< On-board NTC thermistor (ADC-based)
    RTC_temp_sensor        = 1  ///< DS3231 RTC internal temperature sensor
};

////////////////////////////PIN DEFINITIONS///////////////////////

/**
 * @brief Arduino library for the Margay data logger.
 * @details Provides timed SD card logging, on-board diagnostics (BME280,
 * battery voltage, RTC), external sensor I2C management, sleep/wake via
 * RTC alarm, manual log button, and optional external interrupt counting.
 *
 * Typical usage:
 * @code
 * Margay Logger(MODEL_3v0);
 * void setup() { Logger.begin(I2CVals, sizeof(I2CVals), header); }
 * void loop()  { Logger.run(update, 60); }
 * @endcode
 */
class Margay
{

  public:
    /**
     * @brief Instantiate a Margay logger object.
     * @details Sets board-specific pin assignments based on model and build.
     * Does not initialise hardware; call begin() in setup().
     * @param model_ Hardware model version (default MODEL_3v0).
     * @param specs_ Build variant selecting on-board ADC address
     *               (default BUILD_B).
     */
    Margay(board model_ = MODEL_3v0, build specs_ = BUILD_B);

    /**
     * @brief Initialise the logger with a list of external I2C sensor addresses.
     * @details Powers on hardware, initialises the RTC, SD card, BME280, and
     * ADC. Reads the serial number from EEPROM. Runs self-tests for I2C
     * devices, clock, SD card, battery, and power rail, reporting results
     * over Serial at 38400 baud and signalling status via the RGB LED.
     * Attaches RTC alarm and manual-log-button interrupts.
     * Accepts an optional time string over Serial (format YYMMDDHHMMSS) to
     * set the RTC clock on first use.
     * @param vals Pointer to array of 7-bit I2C addresses of external sensors.
     * @param numVals Number of addresses in vals. Silently truncated to 128.
     * @param header_ Comma-separated column header string for the log file,
     *                matching the CSV data returned by the user's update()
     *                function.
     */
    void begin(uint8_t *vals, uint8_t numVals, String header_);

    /**
     * @brief Initialise the logger with no external I2C sensors.
     * @details Convenience overload; equivalent to calling
     * begin(empty_array, 0, header_). Logs only on-board sensor values.
     * @param header_ Optional column header string (default empty).
     */
    void begin(String header_ = "");

    /**
     * @brief Write a string to the SD card log file and echo it to Serial.
     * @param val String to log.
     * @return 0 on success, -1 if the log file could not be opened.
     */
    int logStr(String val);

    /**
     * @brief Set the on-board RGB LED to a packed color value.
     * @details The color format is 0xLLRRGGBB: byte 3 = luminosity,
     * byte 2 = red, byte 1 = green, byte 0 = blue. Use the predefined
     * color constants (RED, GREEN, BLUE, etc.) or OFF to turn the LED off.
     * @param val Packed 32-bit color value.
     */
    void LED_Color(unsigned long val);

    /**
     * @brief Main logging loop; call from Arduino loop().
     * @details Handles three logging triggers:
     *   - RTC alarm (every logInterval seconds): logs a data point and
     *     resets the alarm.
     *   - Manual log button press: logs an additional data point immediately.
     *   - External interrupt (if configured via setExtInt()): increments the
     *     event counter.
     * Puts the MCU into SLEEP_MODE_PWR_DOWN between events to minimise
     * power consumption. Turns the SD card and auxiliary power rail off
     * during sleep and restores them on wake.
     * @param f Pointer to the user's update() function, which must return a
     *          comma-separated String of sensor readings with a trailing comma.
     * @param logInterval Logging interval in seconds.
     */
    void run(String (*f)(void), unsigned long logInterval);

    /**
     * @brief Read voltage from the on-board MCP3421 ADC.
     * @details Only available on BUILD_B, BUILD_C, and BUILD_D. Temporarily
     * switches the I2C bus to the internal bus, takes a reading, and restores
     * the bus to its prior state.
     * @return Voltage in volts, or 0 if no on-board ADC is present.
     */
    float getVoltage();

    /**
     * @brief Log one data point immediately, outside the normal run() cycle.
     * @details Switches the I2C bus to external, calls the user's update()
     * function to obtain sensor data, restores the bus, prepends on-board
     * values (timestamp, BME280, RTC temp, battery voltage), and writes the
     * complete row to the SD card.
     * @param update Pointer to the user's update() function.
     */
    void addDataPoint(String (*update)(void));

    /**
     * @brief Create a new sequentially numbered log file on the SD card.
     * @details Log files are stored at SD:/NW/<SN>/Logs/LogNNNNN.txt.
     * Searches for the next unused number and writes the library version,
     * serial number, and column header as the first two lines.
     * Called automatically by run() when a new log is started; can also be
     * called directly (e.g. from HighSpeed_NoSleep sketches).
     */
    void initLogFile();

    /**
     * @brief Configure a pin as an external interrupt event counter.
     * @details Attaches a falling-edge interrupt to the given pin. Each
     * falling edge increments an internal counter accessible via
     * getExtIntCount(). Intended for pulse-output sensors such as tipping
     * bucket rain gauges and anemometers.
     * Must be called before begin().
     * @param n Arduino pin number for the external interrupt.
     * @param header_entry CSV column label for the counter, including trailing
     *                     comma (default "nInterrupts,").
     */
    void setExtInt(uint8_t n, String header_entry = "nInterrupts,");

    /**
     * @brief Atomically read the external interrupt event count.
     * @details Disables interrupts while reading and optionally resetting the
     * 16-bit counter to prevent torn reads on the 8-bit AVR. Any interrupt
     * that arrives during the critical section is deferred, not lost.
     * @param reset0 If true (default), reset the counter to zero after reading.
     * @return Number of external interrupt events since the last reset.
     */
    uint16_t getExtIntCount(bool reset0 = true);

    /**
     * @brief Atomically set the external interrupt counter to a given value.
     * @details Disables interrupts during the write to prevent a torn store
     * on the 8-bit AVR. Any interrupt that arrives during the critical section
     * is deferred, not lost.
     * @param start Value to set the counter to (default 0).
     */
    void resetExtIntCount(uint16_t start = 0);

    /**
     * @brief Read temperature from an on-board sensor.
     * @param sensor Temperature source. Options:
     *               - thermistor_temp_sensor (0): on-board NTC thermistor
     *               - RTC_temp_sensor (1): DS3231 RTC internal sensor
     *               Defaults to thermistor_temp_sensor.
     * @return Temperature in degrees Celsius, or -1234 on invalid input.
     */
    float getTemp(temp_source sensor = thermistor_temp_sensor);

    /**
     * @brief Read battery pack voltage.
     * @details Reads the voltage divider on BatSense_Pin and applies a
     * compensation factor derived from the on-board voltage reference.
     * The 3.3 V regulator is used as the ADC reference.
     * @return Battery pack voltage in volts.
     */
    float getBatVoltage();

    /**
     * @brief Estimate remaining battery charge as a percentage.
     *
     * Uses a quadratic fit developed for Duracell AA alkaline cells.
     * Should generalise well to most alkalines; accuracy not guaranteed
     * for other chemistries. Accurate to within ~1% from 30% to 100%
     * capacity at 25°C. Configure the number of cells in series via
     * the public member variable NCells (default 3). NCells must be > 0;
     * setting it to 0 will print an error to Serial and return -1.
     *
     * @return Charge percentage in [0, 100], or -1 if NCells == 0.
     */
    float getBatPercentage();

    /**
     * @brief Initialise the on-board MCP3421 ADC at a given resolution.
     * @details Only meaningful on BUILD_B, BUILD_C, and BUILD_D. Called
     * automatically by begin() for those builds; exposed publicly for
     * advanced use in high-speed sketches.
     * @param desiredResolution ADC resolution in bits (12, 14, 16, or 18).
     */
    void initADC(uint8_t desiredResolution);

    /**
     * @brief Pulse the external watchdog timer's DONE pin.
     * @details Pulses WDHold HIGH for 5 µs to feed the hardware watchdog.
     * Has no effect on board models without a watchdog timer (WDHold == 255).
     * Called automatically by run() after each logging event.
     */
    void resetWDT();

    /**
     * @brief Control the on-board 3.3 V power rail.
     * @details Drives the BatSwitch pin to connect or disconnect the battery
     * from the on-board sense circuitry. Has no effect on board models
     * without a battery switch (WDHold == 255 models).
     * @param state ON to enable, OFF to disable.
     */
    void powerOB(bool state);

    /**
     * @brief Control the auxiliary 3.3 V power rail for external sensors.
     * @details Drives Ext3v3Ctrl to enable or disable power to the external
     * sensor connector. Logic is inverted on MODEL_0v0/1v0 relative to
     * MODEL_2v0 and later. Enabling auxiliary power also enables on-board
     * power via powerOB().
     * @param state ON to enable, OFF to disable.
     */
    void powerAux(bool state);

    // -----------------------------------------------------------------------
    // Public pin definitions
    // These are initialised by the constructor to the correct values for the
    // selected board model and build. Advanced users may read these to
    // determine hardware assignments; do not modify after begin() is called.
    // -----------------------------------------------------------------------

    uint8_t SD_CS  = 4;  ///< SD card SPI chip-select pin.
    uint8_t AuxLED = 20; ///< Auxiliary single-color LED pin.
    uint8_t RedLED = 13; ///< Red channel of on-board RGB LED (active low).
    uint8_t GreenLED = 15; ///< Green channel of on-board RGB LED (active low).
    uint8_t BlueLED  = 14; ///< Blue channel of on-board RGB LED (active low).

    uint8_t VRef_Pin      = 2; ///< ADC pin connected to voltage reference.
    uint8_t ThermSense_Pin = 1; ///< ADC pin connected to on-board NTC thermistor divider.
    uint8_t BatSense_Pin   = 0; ///< ADC pin connected to battery voltage divider.

    uint8_t VSwitch_Pin = 3; ///< Voltage switch control pin.
    uint8_t SD_CD       = 1; ///< SD card detect pin (LOW when card is present).

    uint8_t Ext3v3Ctrl = 19; ///< Enable pin for the auxiliary 3.3 V sensor power rail.
    uint8_t I2C_SW     = 21; ///< I2C bus switch (HIGH = external bus, LOW = internal bus).
    uint8_t PG         = 18; ///< Power-good indicator pin.
    uint8_t ExtInt     = 11; ///< External interrupt pin (legacy; use setExtInt()).
    uint8_t RTCInt     = 10; ///< RTC alarm interrupt pin.
    uint8_t LogInt     =  2; ///< Manual log button interrupt pin.
    uint8_t WDHold     = 23; ///< Watchdog timer DONE pin (255 = not present on this model).
    uint8_t BatSwitch  = 22; ///< Battery switch control pin (255 = not present on this model).
    uint8_t TX         = 11; ///< Hardware Serial1 TX pin (sensor-facing UART).
    uint8_t RX         = 10; ///< Hardware Serial1 RX pin (sensor-facing UART).
    uint8_t D0         =  3; ///< General-purpose digital I/O pin 0.

    // -----------------------------------------------------------------------
    // Public configuration variables
    // May be modified between begin() and the first call to run().
    // -----------------------------------------------------------------------

    uint8_t NCells = 3; ///< Number of AA cells in series in the battery pack. Used by getBatPercentage().
    float BatVoltageError      = 3.3; ///< Battery voltage threshold [V] below which BatError is set and logged.
    float BatPercentageWarning = 50;  ///< Battery charge threshold [%] below which BatWarning is set and logged.
    const String LibVersion = "1.2.0"; ///< Library version string, written to every log file header.

  protected:
    float tempConvert(float V, float vcc, float R,
        float A, float B, float C, float D, float R25);
    void blinkGood();
    virtual void writeDataToSD();
    virtual void buttonLog();
    static void isr0();
    static void isr1();
    static void isr2();
    static Margay* selfPointer;
    static void dateTimeSD(uint16_t* date, uint16_t* time);
    void switchExternalI2C(bool desiredState);
    void sleepNow();
    void turnOffSDcard();
    void turnOnSDcard();
    void getTime();
    String getOnBoardVals();
    void I2Ctest();
    void SDtest();
    void clockTest();
    void batTest();
    void powerTest();
    void bme280Readings();
    void extIntCounter();
    void _addDataPoint(String data);
    void farmGateI2C(bool initialStateExternalI2C);

    DS3231_Logger RTC;
    MCP3421 adc;
    BME bme280;

    float A = 0.003354016;
    float B = 0.0003074038;
    float C = 1.019153E-05;
    float D = 9.093712E-07;
    String LogTimeDate = "2063/04/05 20:00:00";
    bool i2cTruncated = false; // true if numVals passed to begin() exceeded I2C_ADR capacity
    bool OnBoardError = false;
    bool SensorError = false;
    bool TimeError = false;
    bool SDCardMissing = false;
    bool BatError = false;
    bool BatWarning = false;
    String Header = "";
    const char HexMap[16] = {
      '0', '1', '2', '3', '4', '5', '6', '7',
      '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    }; // hex digit lookup table
    char SN[20] = {0}; // serial number: 19 chars + null terminator
    uint8_t NumADR = 0;
    uint8_t I2C_ADR[128] = {0}; // one slot per usable 7-bit I2C address
    uint8_t NumADR_OB = 1;
    uint8_t I2C_ADR_OB[2] = {0x68}; //ADC, Clock

    float BatteryDivider = 2.0; //Default for v1.0

    board Model;
    build Specs;

    volatile bool LogEvent = false; //Used to test if logging should begin yet
    volatile bool NewLog = false; //Used to tell system to start a new log
    volatile int AwakeCount = 0;

    char FileNameC[13]; // "LogNNNNN.txt" (12 chars) + null terminator
    char FileNameTestC[11]; // "HWTest.txt" (10 chars) + null terminator
    bool externalI2COn = false;
    SdFat SD;
    byte  keep_SPCR;
    byte keep_ADCSRA;
};

#endif
