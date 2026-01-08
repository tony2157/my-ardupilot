/*
 * AP_ARRC_LB5900 - LadyBug LB5900 Series True-RMS Power Sensor Driver
 * =====================================================================
 *
 * DESCRIPTION:
 *   ArduPilot driver for LadyBug LB5900 Series True-RMS Power Sensors.
 *   These sensors use SCPI (Standard Commands for Programmable Instruments)
 *   commands over I2C for RF power measurement applications.
 *
 *   Key sensor features:
 *   - No-Zero No-Cal system (no calibration required)
 *   - Full dynamic range measurement with each sample
 *   - Configurable measurement rate and averaging
 *   - High accuracy from noise floor to maximum power level
 *
 * IMPLEMENTATION:
 *   This driver operates the sensor in free-run mode (INIT:CONT ON) using
 *   the FETCh? SCPI command. In this mode, the sensor continuously averages
 *   measurements in a circular buffer, and FETCh? returns the most recent
 *   "trailing" average when queried.
 *
 *   I2C Communication Protocol:
 *   - Header 0x06: Next read returns status and length
 *   - Header 0x0C: Next read returns complete output buffer
 *   - Data format: Big-endian, ASCII response (e.g., "-6.38E+01" dBm)
 *
 *   Measurement sequence (per timer callback):
 *   1. Send FETCh? command with header 0x06
 *   2. Read 4 bytes (status + length)
 *   3. Send header 0x0C with buffer length
 *   4. Read ASCII power measurement
 *   5. Convert to float (dBm)
 *
 * CONFIGURATION TABLES:
 *
 *   Measurement Rate (MRATe) Settings:
 *   +-------+----------+----------------+------------------+
 *   | rate  | Mode     | Time/Average   | Averaging Range  |
 *   +-------+----------+----------------+------------------+
 *   |   0   | NORMAL   | 38.4 ms        | 1 - 1024         |
 *   |   1   | DOUBLE   | 19.6 ms        | 1 - 1024         |
 *   |   2   | FAST     |  3.2 ms        | Forced to 1      |
 *   |   3   | SUPER    |  1.6 ms        | 1 - 1024         |
 *   +-------+----------+----------------+------------------+
 *
 *   Total Measurement Time = Time/Average x avg_cnt
 *
 *   Example Configurations:
 *   +----------+---------+------------------+------------------+
 *   | rate     | avg_cnt | Measurement Time | Update Rate      |
 *   +----------+---------+------------------+------------------+
 *   | NORMAL=0 |    1    |  38.4 ms         |  ~26 Hz          |
 *   | NORMAL=0 |    5    | 192.0 ms         |  ~5 Hz           |
 *   | DOUBLE=1 |    1    |  19.6 ms         |  ~51 Hz          |
 *   | DOUBLE=1 |    5    |  98.0 ms         |  ~10 Hz          |
 *   | FAST=2   |   (1)   |   3.2 ms         | ~312 Hz          |
 *   | SUPER=3  |    1    |   1.6 ms         | ~625 Hz          |
 *   | SUPER=3  |    4    |   6.4 ms         | ~156 Hz          |
 *   +----------+---------+------------------+------------------+
 *   Note: FAST mode forces avg_cnt to 1 regardless of user setting.
 *
 *   Frequency Range:
 *   - Valid range: 1 MHz to 18000 MHz (18 GHz)
 *   - Set to match your RF signal frequency for best accuracy
 *
 *   I2C Configuration:
 *   - Default address: 0x4C
 *   - Bus 0: Pixhawk 2.1 I2C
 *   - Bus 1: Pixhawk 1 / PixRacer I2C
 *
 * USAGE EXAMPLE:
 *
 *   AP_ARRC_LB5900 power_sensor;
 *
 *   // Initialize: Bus 0, Address 0x4C, 2400 MHz, 5 averages, DOUBLE rate
 *   if (power_sensor.init(0, 0x4C, 2400, 5, 1)) {
 *       // Sensor ready
 *   }
 *
 *   // Read power (returns dBm as float)
 *   if (power_sensor.healthy()) {
 *       float power_dBm = power_sensor.power_measure();
 *   }
 *
 * REFERENCES:
 *   - LadyBug LB5900 Programmatic Measurement Commands and Examples V3.1
 *   - LadyBug LB5900 Programming Guide
 *
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define LADYBUG_I2C_BASE_ADDRESS    0x4C
#define LADYBUG_TIMEOUT_MAX         0x6000

class AP_ARRC_LB5900 {
public:
    AP_ARRC_LB5900(void);
    ~AP_ARRC_LB5900(void){}

    /*
     * Initialize the LB5900 power sensor
     * @param busId    I2C bus number (0 = Pixhawk 2.1, 1 = Pixhawk 1/PixRacer)
     * @param i2cAddr  I2C address (default 0x4C)
     * @param freq     RF frequency in MHz (1 - 18000)
     * @param avg_cnt  Number of averages (1 - 1024, ignored in FAST mode)
     * @param rate     Measurement rate: 0=NORMAL, 1=DOUBLE, 2=FAST, 3=SUPER
     * @return true if initialization successful
     */
    bool init(uint8_t busId, uint8_t i2cAddr, uint16_t freq, uint8_t avg_cnt, uint8_t rate);

    // Get the last measured power in dBm
    float power_measure(void) { return _power; }

    // Returns true if sensor is responding and providing valid readings
    bool healthy(void) { return _healthy; }

    // Returns true if sensor has been successfully initialized
    bool has_init(void) { return _initialized; }

    // Change the I2C address at runtime
    void set_i2c_addr(uint8_t addr);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    HAL_Semaphore _sem;                     // Semaphore for thread-safe access
    float _power;                           // Last measured power in dBm
    bool _healthy;                          // Sensor communication status
    bool _initialized;                      // Initialization complete flag
    uint32_t Sensor_TimeOut;                // I2C timeout counter
    uint8_t commandNumber;                  // Command sequence tracker for configuration

    union{
        uint8_t byte[196];
        struct
        {
            uint8_t commandAndLength[4];
            uint8_t buffer[192];
        }field;
    }config_sensor_buffer;

    union{
        uint8_t byte[68];
        struct
        {
            uint8_t commandAndLength[4];
            uint8_t buffer[64];
        }field;
    }write_sensor_buffer;

    union{
        uint8_t byte[100];
        char string[100];
        struct
        {
            uint8_t statusAndLength[4];
            uint8_t buffer[96];
        }field;
    }read_sensor_buffer;

    union
    {
        uint32_t  ui;
        uint8_t c[4];
    }header;

    union
    {
        uint32_t  ui;
        uint8_t c[4];
    }bufLength;

    typedef enum nextReadType
    {
        nextReadIsStatusAndLength		= 0x06,
        nextReadIsCompleteOutputBuffer	= 0x0C,
        nextReadIsPartialOutputBuffer 	= 0x18, // Not implemented yet
        nextReadIsNULL					= 0xF0
    }nextReadType_t;

    bool configSensor(uint16_t freq, uint8_t avg_cnt, uint8_t rate);
    bool _fetch_and_read(void);  // Combined FETCh? and read operation
    void _timer(void);
    uint32_t _calculate_measurement_period_us(uint8_t rate, uint8_t avg_cnt);

    // Configuration storage for timing calculations
    uint8_t _avg_cnt;
    uint8_t _rate;
};