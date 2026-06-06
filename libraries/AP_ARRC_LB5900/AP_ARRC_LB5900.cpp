#include "AP_ARRC_LB5900.h"
#include <utility>
#include <stdio.h>
#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AP_ARRC_LB5900::AP_ARRC_LB5900() :
    _dev(nullptr),
    _power(0),
    _healthy(false),
    _initialized(false),
    _avg_cnt(1),
    _rate(0)
{
}

bool AP_ARRC_LB5900::init(uint8_t busId, uint8_t i2cAddr, uint16_t freq, uint8_t avg_cnt, uint8_t rate)
{
    commandNumber = 0;
    Sensor_TimeOut = 200;

    // Store configuration for timing calculations
    _rate = (rate > 3) ? 3 : rate;
    _avg_cnt = (_rate == 2) ? 1 : avg_cnt;  // FAST mode forces avg_cnt to 1

    // Bus 0 is for Pixhawk 2.1 I2C and Bus 1 is for Pixhawk 1 and PixRacer I2C
    // Check if device exists
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    if (!_dev) {
        _healthy = false;
        return false;
    }
    _healthy = true;

    hal.scheduler->delay(50);

    _dev->get_semaphore()->take_blocking();

    _dev->set_retries(5);

    // Start the first measurement
    uint16_t iter = 0;
    while(!configSensor(freq, avg_cnt, rate)) {
        hal.scheduler->delay(3);
        if (iter == 100){
            _healthy = false;
            _dev->get_semaphore()->give();
            return false;
        }
        iter++;
    }

    _initialized = true;

    _dev->get_semaphore()->give();

    // Calculate callback period based on MRATe and averaging count
    // Add 10% margin to ensure measurement is ready
    uint32_t measurement_period_us = _calculate_measurement_period_us(_rate, _avg_cnt);
    measurement_period_us = (measurement_period_us * 11) / 10;  // Add 10% margin

    // Clamp to reasonable bounds (minimum 5ms, maximum 500ms).
    // Unsigned literals so MAX/MIN don't compare signed/unsigned (-Werror=sign-compare).
    measurement_period_us = MAX(measurement_period_us, 5000U);
    measurement_period_us = MIN(measurement_period_us, 500000U);

    _dev->register_periodic_callback(measurement_period_us, FUNCTOR_BIND_MEMBER(&AP_ARRC_LB5900::_timer, void));

    return true;
}

void AP_ARRC_LB5900::set_i2c_addr(uint8_t addr)
{
    if (_dev) {
        _dev->set_address(addr);
    }
}


bool AP_ARRC_LB5900::configSensor(uint16_t freq, uint8_t avg_cnt, uint8_t rate)
{
    // Available sensor reading rate
    const char* (mrate[1])[4] = 
    {
        "NORMAL",   // 20 readings per sec
        "DOUBLE",   // 40 readings per sec
        "FAST",     // 110 readings per sec (disallows average count)
        "SUPER"     // 110 readings per sec (allows average count)
    };
    if (rate > 3) rate = 3;
    if (rate == 2) avg_cnt = 1;
    if (freq > 18000) freq = 18000;

    // Buffers must fit the full built command including the trailing NUL:
    //   FREQ:    "FREQ " (5) + up to 5 digits + " MHZ" (4) + NUL  = 15
    //   AVG_CNT: "SENS:AVER:COUN " (15) + up to 3 digits + NUL    = 19
    //   MRATE:   "SENS:MRAT " (10) + up to 6 chars (e.g. NORMAL)  = 17
    // (the previous sizes overflowed on strcat, e.g. "FREQ 3000 MHZ" into FREQ[11])
    char FREQ[16] = "FREQ ";
    char AVG_CNT[20] = "SENS:AVER:COUN ";
    char MRATE[20] = "SENS:MRAT ";
    char temp[6];

    // Convert user params freq, avg_cnt and mrate to strings
    snprintf(temp,6,"%d",freq);
    strcat(FREQ, temp);
    strcat(FREQ, " MHZ");
    snprintf(temp,6,"%d",avg_cnt);
    strcat(AVG_CNT, temp);
    strcat(MRATE, mrate[0][rate]);

    // List of initial commands to configure the LB5900
    const char* (cmd[1])[10] = 
    {
        "SYST:PRES DEF",
        "AVER:COUN:AUTO OFF",
        "SENS:AVER:SDET OFF",
        "INIT:CONT ON",
        MRATE,
        AVG_CNT,
        FREQ,
        "\0" // STOP LIST
    };

    // Send initial commands through I2C
    while(1){
        if(strlen(cmd[0][commandNumber])  != 0 ){

            // Build header
            memset(config_sensor_buffer.byte, 0x00, 196);
            config_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsStatusAndLength;
            header.ui = strlen(cmd[0][commandNumber]) + 5; // Add one for terminator and 4 for header
            config_sensor_buffer.field.commandAndLength[3] = header.c[0];
            config_sensor_buffer.field.commandAndLength[2] = header.c[1];
            config_sensor_buffer.field.commandAndLength[1] = header.c[2];
            // Add command to buffer
            strcpy((char*)config_sensor_buffer.field.buffer, cmd[0][commandNumber]);
            // Send Command with "nextReadIsStatusAndLength" header
            if(!_dev->transfer(config_sensor_buffer.byte, header.ui, nullptr, 0)) {
                return false;
            }
            commandNumber++;
        }
        else{
            commandNumber = 0;
            hal.scheduler->delay(1);
            return true;
        }
    }
}

uint32_t AP_ARRC_LB5900::_calculate_measurement_period_us(uint8_t rate, uint8_t avg_cnt)
{
    // Per LB5900 documentation Section IV - Time per Average based on MRATe setting:
    // NORMAL: 38.4ms, DOUBLE: 19.6ms, FAST: 3.2ms (forced to 1 avg), SUPER: 1.6ms
    uint32_t time_per_avg_us;
    switch(rate) {
        case 0:  // NORMAL
            time_per_avg_us = 38400;
            break;
        case 1:  // DOUBLE
            time_per_avg_us = 19600;
            break;
        case 2:  // FAST (averaging forced to 1)
            time_per_avg_us = 3200;
            avg_cnt = 1;
            break;
        case 3:  // SUPER
            time_per_avg_us = 1600;
            break;
        default:
            time_per_avg_us = 38400;  // Default to NORMAL
            break;
    }

    // Total measurement time = time_per_average * number_of_averages
    return time_per_avg_us * avg_cnt;
}

bool AP_ARRC_LB5900::_fetch_and_read(void)
{
    // Phase 1 & 2: Combined and optimized FETCh? and read operation
    // Correct sequence: Send FETCh? command first, then read the response

    // Clear headers
    bufLength.ui = 0;
    header.ui = 0;

    // Step 1: Send FETCh? command with header 0x06 (nextReadIsStatusAndLength)
    // This requests the current measurement from the sensor's circular buffer
    const char* fetch_cmd = "FETCh?";
    memset(write_sensor_buffer.byte, 0x00, 68);
    write_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsStatusAndLength;
    header.ui = strlen(fetch_cmd) + 5;  // Command length + terminator + 4 byte header
    write_sensor_buffer.field.commandAndLength[3] = header.c[0];
    write_sensor_buffer.field.commandAndLength[2] = header.c[1];
    write_sensor_buffer.field.commandAndLength[1] = header.c[2];
    strcpy((char*)write_sensor_buffer.field.buffer, fetch_cmd);

    if(!_dev->transfer(write_sensor_buffer.byte, header.ui, nullptr, 0)) {
        return false;
    }

    // Step 2: Read 4 bytes status and length
    // Reduced delay - I2C bus at 400kHz should be ready quickly
    header.ui = 0;
    if(!_dev->transfer(nullptr, 0, header.c, 4)) {
        return false;
    }

    // Parse status and length (big-endian to little-endian conversion)
    bufLength.c[0] = header.c[3];
    bufLength.c[1] = header.c[2];
    bufLength.c[2] = header.c[1];
    bufLength.c[3] = 0;

    // Phase 1 fix: Corrected bitwise AND operator (was && which is logical AND)
    // Check status bit 4 (0x10) indicates data ready
    if((header.c[0] & 0x10) && (bufLength.ui != 0))
    {
        // Step 3: Request the complete output buffer using header 0x0C
        memset(write_sensor_buffer.byte, 0x00, 68);
        write_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsCompleteOutputBuffer;
        write_sensor_buffer.field.commandAndLength[1] = bufLength.c[2];
        write_sensor_buffer.field.commandAndLength[2] = bufLength.c[1];
        write_sensor_buffer.field.commandAndLength[3] = bufLength.c[0];

        if(!_dev->transfer(write_sensor_buffer.byte, 4, nullptr, 0)) {
            return false;
        }

        // Step 4: Read the measurement data
        memset(read_sensor_buffer.byte, 0x00, 100);
        if(!_dev->transfer(nullptr, 0, read_sensor_buffer.byte, bufLength.ui)) {
            return false;
        }

        // Convert ASCII response to float (e.g., "-6.38395844E+01" -> -63.8395844)
        _power = strtof(read_sensor_buffer.string, nullptr);

        return true;
    }

    return false;
}

void AP_ARRC_LB5900::_timer(void)
{
    WITH_SEMAPHORE(_sem);
    // Phase 1 fix: Correct order - FETCh? is now sent first within _fetch_and_read()
    // In free-run mode (INIT:CONT ON), the sensor continuously averages measurements.
    // FETCh? returns the most recent trailing average from the circular buffer.
    _healthy = _fetch_and_read();
}