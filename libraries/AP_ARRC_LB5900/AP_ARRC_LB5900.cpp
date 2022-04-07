#include "AP_ARRC_LB5900.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AP_ARRC_LB5900::AP_ARRC_LB5900() :
    _dev(nullptr),
    //_temperature(0),
    _power(0),
    _healthy(false)
{
}

bool AP_ARRC_LB5900::init(uint8_t busId, uint8_t i2cAddr)
{
    commandNumber = 0;
    Sensor_TimeOut = 200;

    // Bus 0 is for Pixhawk 2.1 I2C and Bus 1 is for Pixhawk 1 and PixRacer I2C
    // Check if device exists
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    if (!_dev) {
        return false;
    }
    _dev->get_semaphore()->take_blocking();

    _dev->set_retries(10);

    // Start the first measurement
    if (!_measure()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    /* Request 20Hz update */
    // Max conversion time is 12 ms
    _dev->register_periodic_callback(100000,
                                     FUNCTOR_BIND_MEMBER(&AP_ARRC_LB5900::_timer, void));
    return true;
}

void AP_ARRC_LB5900::set_i2c_addr(uint8_t addr){
    if (_dev) {
        _dev->set_address(addr);
    }
}


bool AP_ARRC_LB5900::configSensor(uint16_t freq, uint8_t avg_cnt)
{
    while(1){
        if(strlen(example[0][commandNumber])  != 0 ){
            // Build header
            memset(write_sensor_buffer.byte, 0x00, 5000);
            write_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsStatusAndLength;
            header.ui = strlen(example[0][commandNumber]) + 5; // Add one for terminator and 4 for header
            write_sensor_buffer.field.commandAndLength[3] = header.c[0];
            write_sensor_buffer.field.commandAndLength[2] = header.c[1];
            write_sensor_buffer.field.commandAndLength[1] = header.c[2];
            // Add command to buffer
            strcpy((char*)write_sensor_buffer.field.buffer, example[0][commandNumber]);
            // Send Command with "nextReadIsStatusAndLength" header
            while(!_dev->transfer(write_sensor_buffer.byte, header.ui, nullptr, 0)) {
                hal.scheduler->delay(1);
                if (Sensor_TimeOut-- == 0)
                {
                    Sensor_TimeOut = 200;
                    return false;
                }
            }
        }
        else{
            commandNumber = 0;
            return true;
        }

        if(commandNumber++ >= 9)
        {
            commandNumber = 0;
            return false;
        }
    }
}



bool AP_ARRC_LB5900::_measure()
{
    uint8_t cmd = 0x00;
        if (!_dev->transfer(&cmd, 1, nullptr, 0)) {
            return false;
        }
        return true;
}

bool AP_ARRC_LB5900::_collect(float &hum, float &temp)
{
    uint8_t data[4];
    int16_t raw;
    // Read sensors
    if (!_dev->transfer(nullptr, 0, data, sizeof(data))) {
        return false;
    }

    // Verify data with the checksum
    if ((data[0] & 0x40) == 0x40){
        return false;
    }

    WITH_SEMAPHORE(_sem);                           // semaphore for access to shared frontend data
    // Bit shift and convert to floating point number
    raw = (data[0] << 8) | data[1];
    raw = raw & 0x3FFF;

    hum = (100.0 / (powf(2,14) - 1)) * (float)raw;

    data[3] = (data[3] >> 2);
    raw = (data[2] << 6) | data[3];
    temp = (165.0 / (powf(2,14) - 1)) * (float)raw + 233.15f;

    return true;  
}

void AP_ARRC_LB5900::_timer(void)
{
    _healthy = _collect(_humidity, _temperature);   // Retreive data from the sensor
    _measure();                                     // Request a new measurement to the sensor
}