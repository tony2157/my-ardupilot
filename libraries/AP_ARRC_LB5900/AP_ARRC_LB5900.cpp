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
    _ready(false)
{
}

bool AP_ARRC_LB5900::init(uint8_t busId, uint8_t i2cAddr, uint16_t freq, uint8_t avg_cnt)
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

    _dev->set_retries(1);

    // Start the first measurement
    if (!configSensor(freq, avg_cnt)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();

    /* Request 10Hz update */
    // Max conversion time is 12 ms
    _dev->register_periodic_callback(100000,
                                     FUNCTOR_BIND_MEMBER(&AP_ARRC_LB5900::_timer, void));
    return true;
}

void AP_ARRC_LB5900::set_i2c_addr(uint8_t addr)
{
    if (_dev) {
        _dev->set_address(addr);
    }
}


bool AP_ARRC_LB5900::configSensor(uint16_t freq, uint8_t avg_cnt)
{
    char FREQ[10 + sizeof(char)] = "FREQ ";
    char AVG_CNT[12 + sizeof(char)] = "AVER:COUN ";
    char temp[5];

    sprintf(temp, "%d", freq);
    strcat(FREQ, temp);
    strcat(FREQ, " MHZ");

    sprintf(temp, "%d", avg_cnt);
    strcat(AVG_CNT, temp);

    char* (cmd[1])[10] = 
    {
        "SYST:PRES",
        FREQ,
        "AVER:COUN:AUTO 0",
        AVG_CNT,
        "AVER:SDET 0",
        "INIT:CONT 1",
        "\0" // STOP LIST
    };

    while(1){
        if(strlen(cmd[0][commandNumber])  != 0 ){
            // Build header
            memset(write_sensor_buffer.byte, 0x00, 5000);
            write_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsStatusAndLength;
            header.ui = strlen(cmd[0][commandNumber]) + 5; // Add one for terminator and 4 for header
            write_sensor_buffer.field.commandAndLength[3] = header.c[0];
            write_sensor_buffer.field.commandAndLength[2] = header.c[1];
            write_sensor_buffer.field.commandAndLength[1] = header.c[2];
            // Add command to buffer
            strcpy((char*)write_sensor_buffer.field.buffer, cmd[0][commandNumber]);
            // Send Command with "nextReadIsStatusAndLength" header
            while(!_dev->transfer(write_sensor_buffer.byte, header.ui, nullptr, 0)) {
                hal.scheduler->delay(1);
                if (Sensor_TimeOut-- == 0)
                {
                    Sensor_TimeOut = 2;
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

bool AP_ARRC_LB5900::_read(void)
{   
    // Clear headers
    bufLength.ui = 0;
    header.ui = 0;

    // Send Prepare Status & Length (no command) using header O6h
    memset(write_sensor_buffer.byte, 0x00, 5000);
    write_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsStatusAndLength;
    header.ui = 4; // Only 4 bytes are sent (without command)
    write_sensor_buffer.field.commandAndLength[3] = header.c[0];
    write_sensor_buffer.field.commandAndLength[2] = header.c[1];
    write_sensor_buffer.field.commandAndLength[1] = header.c[2];
    while(!_dev->transfer(write_sensor_buffer.byte, header.ui, nullptr, 0)) {
        hal.scheduler->delay(1);
        if (Sensor_TimeOut-- == 0)
        {
            Sensor_TimeOut = 2;
            return false;
        }
    }

    // Read 4 bytes from the sensor. This contains the status byte and 3 length bytes.
    // header.c is used as temp variable
    while(!_dev->transfer(nullptr, 0, header.c, 4)) {
        hal.scheduler->delay(50);
        if (Sensor_TimeOut-- == 0)
        {
            Sensor_TimeOut = 200;
            return false;
        }
    }

    // Transfer status&length to read_buffer
    bufLength.c[0] = header.c[3];
    bufLength.c[1] = header.c[2];
    bufLength.c[2] = header.c[1];

    // If status is good, get the data!
    if(bufLength.ui != 0)
    {
        // Write the number of bytes to the sensor that are to be read back using header 0Ch
        write_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsCompleteOutputBuffer;
        write_sensor_buffer.field.commandAndLength[1] = 0;
        write_sensor_buffer.field.commandAndLength[2] = 0;
        write_sensor_buffer.field.commandAndLength[3] = 4;

        while(!_dev->transfer(write_sensor_buffer.byte, 4, nullptr, 0)) {
            hal.scheduler->delay(1);
            if (Sensor_TimeOut-- == 0)
            {
                Sensor_TimeOut = 2;
                return false;
            }
        }

        // Read back the measurement
        memset(read_sensor_buffer.byte, 0x00, 5000);
        while(!_dev->transfer(nullptr, 0, read_sensor_buffer.byte, bufLength.ui)) {
            hal.scheduler->delay(50);
            if (Sensor_TimeOut-- == 0)
            {
                Sensor_TimeOut = 200;
                return false;
            }
        }

        return true;
    }
    else{
        return false;
    }
    

}

bool AP_ARRC_LB5900::_measure(void)
{
    char* (cmd[1])[10] = 
    {
        "FETCH?",
        "\0" // STOP LIST
    };

    while(1){
        if(strlen(cmd[0][commandNumber])  != 0 ){
            // Build header
            memset(write_sensor_buffer.byte, 0x00, 5000);
            write_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsStatusAndLength;
            header.ui = strlen(cmd[0][commandNumber]) + 5; // Add one for terminator and 4 for header
            write_sensor_buffer.field.commandAndLength[3] = header.c[0];
            write_sensor_buffer.field.commandAndLength[2] = header.c[1];
            write_sensor_buffer.field.commandAndLength[1] = header.c[2];
            // Add command to buffer
            strcpy((char*)write_sensor_buffer.field.buffer, cmd[0][commandNumber]);
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

bool AP_ARRC_LB5900::_compute(void)
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