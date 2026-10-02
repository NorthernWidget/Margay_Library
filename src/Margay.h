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
#include <NW_Logger.h>   // the logger core that Margay and Okapi share (NW_Sensor and NW_Pages through it)

// Build identity: this library's version (held equal to library.properties by
// NW-Tests/version_check.py) and its build commit, set by the NW-Build wrapper from
// git and blank in an Arduino IDE build; the sketch's commit the same way.
#define MARGAY_LIBRARY_VERSION "1.2.0"
#ifndef MARGAY_LIBRARY_COMMIT
#define MARGAY_LIBRARY_COMMIT ""
#endif
#ifndef SKETCH_COMMIT
#define SKETCH_COMMIT ""
#endif
#include "MCP3421.h"

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
class Margay : public NW_Logger
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
    bool begin(uint8_t *vals, uint8_t numVals, String header_) override;
    using NW_Logger::begin; ///< begin(header) with no external sensors

    /** @brief "Margay". */
    const char* name() const override { return "Margay"; }
    /**
     * @brief The logger's own status line: name, serial, hardware version, library
     * version, report code and note word, Pages 0-2 in hex (its reading of itself:
     * battery, onboard environment, clock). The same columns as a sensor's.
     */
    size_t printStatus(Print& out, bool boot = false) override;

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
    void addDataPoint(String (*update)(void)) override;

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

    uint8_t VRef_Pin      = 2; ///< ADC pin connected to voltage reference.
    uint8_t ThermSense_Pin = 1; ///< ADC pin connected to on-board NTC thermistor divider.
    uint8_t BatSense_Pin   = 0; ///< ADC pin connected to battery voltage divider.

    uint8_t VSwitch_Pin = 3; ///< Voltage switch control pin.

    uint8_t Ext3v3Ctrl = 19; ///< Enable pin for the auxiliary 3.3 V sensor power rail.
    uint8_t PG         = 18; ///< Power-good indicator pin.
    uint8_t ExtInt     = 11; ///< External interrupt pin (legacy; use setExtInt()).
    uint8_t BatSwitch  = 22; ///< Battery switch control pin (255 = not present on this model).
    uint8_t TX         = 11; ///< Hardware Serial1 TX pin (sensor-facing UART).
    uint8_t RX         = 10; ///< Hardware Serial1 RX pin (sensor-facing UART).
    uint8_t D0         =  3; ///< General-purpose digital I/O pin 0.

    // -----------------------------------------------------------------------
    // Public configuration variables
    // May be modified between begin() and the first call to run().
    // -----------------------------------------------------------------------

    uint8_t nCells = 3; ///< Number of AA cells in series in the battery pack. Used by getBatPercentage().
    float batVoltageError      = 3.3; ///< Battery voltage threshold [V] below which BatError is set and logged.
    float batPercentageWarning = 50;  ///< Battery charge threshold [%] below which BatWarning is set and logged.
    float _onBoardTemp = NW_ERROR;   ///< Thermistor temperature from the last readOnBoard() [C]
    float _rtcTemp = NW_ERROR;       ///< RTC die temperature from the last readOnBoard() [C]
    float _batVoltage = NW_ERROR;    ///< Battery voltage from the last readOnBoard() [V]
    const String libVersion = MARGAY_LIBRARY_VERSION; ///< Library version string, the logger's FW column in the status file.

  protected:
    /**
     * @brief Everything the on-board thermistor measurement is calibrated by.
     * @details One object, because none of it is ever used apart. a, b, c and d
     * are the Steinhart-Hart coefficients and keep the datasheet's letters;
     * seriesResistance is the divider resistor beside the thermistor, and r25
     * the thermistor's nominal resistance at 25 C. Only a, b, c and d come from
     * Page 1 today (0x22-0x31); the other two hold the board's compiled values,
     * and Page 1 has room at 0x35-0x3F if they are ever provisioned.
     */
    struct Thermistor {
      float a;
      float b;
      float c;
      float d;
      float seriesResistance;   ///< [ohm]
      float r25;                ///< [ohm] at 25 C
    };

    float tempConvert(float V, float vcc, const Thermistor& th);
    void sleepNow() override;
    void turnOffSDcard();
    void turnOnSDcard();
    /** @brief Read the on-board channels into _onBoardTemp, _rtcTemp and _batVoltage (and the timestamp). */
    void readOnBoard() override;

    /** @brief Print the logger's own data columns: the timestamp and the on-board sensors. */
    size_t printDataHeader(Print& out) override;

    /** @brief Print those columns' values, as readOnBoard() left them. Takes no reading. */
    size_t printDataRow(Print& out) override;
    void batTest();
    void powerTest();
    void bme280Readings();
    void _addDataPoint(String data);

    MCP3421 adc;

    Thermistor _thermistor = {0.003354016, 0.0003074038, 1.019153E-05, 9.093712E-07, 10000.0, 10000.0};

    float _batteryDivider = 2.0; //Default for v1.0

    board _model;
    build _specs;

    uint8_t chipFaults();    // Margay's chip-fault bits for Block 0: SDCard, Clock, BME280, SensorBus, Battery
    void fillPages();        // Page 2 and 3 from the logger's own readings, then endReading()
};

#endif
