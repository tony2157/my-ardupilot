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
 *   Everything runs in the I2C bus thread: init() only registers the periodic
 *   callback, so a missing sensor never delays the boot. The callback sends the
 *   configuration (all commands in one go, retried every second until the sensor
 *   answers), then reads one measurement per period. When no reading succeeds
 *   for RECONFIGURE_MS the sensor is configured again: a sensor that browned out
 *   or was plugged in late comes back on its own.
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
 *   5. Convert to float (dBm), rejecting values outside MIN_DBM..MAX_DBM
 *      (SCPI not-a-number 9.91E+37 and overload codes included)
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
 *   // Start: Bus 0, Address 0x4C, 2400 MHz, 5 averages, DOUBLE rate. Returns at once;
 *   // the sensor is configured from the I2C bus thread
 *   power_sensor.init(0, 0x4C, 2400, 5, 1);
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

#include "AP_ARRC_LB5900_config.h"

#if AP_ARRC_LB5900_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>

#define LADYBUG_I2C_BASE_ADDRESS    0x4C

class AP_ARRC_LB5900 {
public:
    AP_ARRC_LB5900(void);

    CLASS_NO_COPY(AP_ARRC_LB5900);

    /*
     * Start the LB5900 power sensor. Never blocks: the sensor is configured from the I2C bus thread
     * @param busId    I2C bus number
     * @param i2cAddr  I2C address (default 0x4C)
     * @param freq     RF frequency in MHz (1 - 18000)
     * @param avg_cnt  Number of averages (ignored in FAST mode)
     * @param rate     Measurement rate: 0=NORMAL, 1=DOUBLE, 2=FAST, 3=SUPER
     * @return false if the I2C bus does not exist
     */
    bool init(uint8_t busId, uint8_t i2cAddr, uint16_t freq, uint8_t avg_cnt, uint8_t rate);

    // a measurement: power in dBm, number of measurements read so far, system time it was read
    struct Sample {
        float power_dbm;
        uint32_t count;
        uint32_t time_ms;
    };
    // latest measurement, false if there has been none
    bool get_sample(Sample &sample) const;

    // latest power in dBm (NaN if there has been none) and the number of measurements read so far
    float power_measure(void) const;
    uint32_t sample_count(void) const;

    // true if polling is enabled and a measurement was read within the last few measurement periods
    bool healthy(void) const;

    enum class State : uint8_t {
        STOPPED = 0,        // not started, no I2C bus, or disabled
        CONFIGURING = 1,    // sending the configuration, retried until the sensor answers
        RUNNING = 2,        // configured, reading measurements
    };
    State state(void) const { return _state; }
    // true if the sensor has been configured at least once since boot
    bool ever_configured(void) const { return _ever_configured; }

    // Enable or disable polling. A disabled sensor makes no I2C transfers and is not healthy
    void set_enabled(bool enabled);
    bool enabled(void) const { return _enabled; }

private:
    static constexpr float MIN_DBM = -120.0f;           // plausible readings; beyond are SCPI codes or garbage
    static constexpr float MAX_DBM = 40.0f;
    static constexpr uint32_t CONFIG_RETRY_MS = 1000;   // configuration attempts while the sensor does not answer
    static constexpr uint32_t RECONFIGURE_MS = 2000;    // configure again after this long without a reading

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    mutable HAL_Semaphore _sem;             // protects _sample
    Sample _sample;
    bool _have_sample;
    volatile State _state;
    volatile bool _enabled;
    bool _ever_configured;
    uint32_t _period_ms;                    // measurement period
    uint32_t _last_ok_ms;                   // bus thread: system time of the last good reading
    uint32_t _config_attempt_ms;            // bus thread: system time of the last configuration attempt

    // configuration
    uint16_t _freq;
    uint8_t _avg_cnt;
    uint8_t _rate;

    union{
        uint8_t byte[196];
        struct
        {
            uint8_t commandAndLength[4];
            uint8_t buffer[192];
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

    typedef enum nextReadType
    {
        nextReadIsStatusAndLength		= 0x06,
        nextReadIsCompleteOutputBuffer	= 0x0C,
        nextReadIsPartialOutputBuffer 	= 0x18, // Not implemented yet
        nextReadIsNULL					= 0xF0
    }nextReadType_t;

    bool _send_command(const char *cmd);
    bool _configure(void);
    bool _fetch_and_read(float &power);
    void _timer(void);
    static uint32_t _calculate_measurement_period_us(uint8_t rate, uint8_t avg_cnt);
};

#endif  // AP_ARRC_LB5900_ENABLED
