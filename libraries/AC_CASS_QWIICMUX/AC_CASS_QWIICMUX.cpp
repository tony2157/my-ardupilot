#include "AC_CASS_QWIICMUX.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

QWIICMUX::QWIICMUX() :
    _dev(nullptr)
{}

bool QWIICMUX::init(uint8_t busId, uint8_t i2cAddr)
{
    // Bus 0 is for Pixhawk 2.1 I2C and Bus 1 is for Pixhawk 1 and PixRacer I2C
    // Check if device exists
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    if (!_dev) {
        return false; }
    
    _dev->get_semaphore()->take_blocking();

    _dev->set_retries(10);

    //Returns true if device is present
    //Tests for device ack to I2C address
    //Then tests if device behaves as we expect
    //Leaves with all ports disabled

    // Write to device, expect a return
    // Set port register to a known value
    if (!setPortState(0xA4)) {
        _dev->get_semaphore()->give();
        return false; }

    uint8_t response = getPortState();
    setPortState(0x00);   //Disable all ports
    if (response != 0xA4){ // Make sure we got back what we expected
        return false; }

    // lower retries for run
    _dev->set_retries(2);

    _dev->get_semaphore()->give();

    /* Request 20Hz update */
    // Max conversion time is 12 ms
    // _dev->register_periodic_callback(100000,
    //                                  FUNCTOR_BIND_MEMBER(&QWIICMUX::_timer, void));
    return true;
}

bool QWIICMUX::setPort(uint8_t portNumber)
{
    uint8_t portValue = 0;

    if (portNumber > 7){
      portValue = 0; } //If port number is out of range, turn off all ports
    else {
      portValue = 1 << portNumber; 
    }
    
    _dev->get_semaphore()->take_blocking();
    if (!_dev->transfer(&portValue, sizeof(portValue), nullptr, 0)) {
            _dev->get_semaphore()->give();
            return false; //Device did not ACK
        }
    _dev->get_semaphore()->give();

    return true;
}

//Returns the first port number bit that is set
//Returns 255 if no port is enabled
uint8_t QWIICMUX::getPort()
{
    uint8_t portBits;
    //Read the current mux settings
    _dev->get_semaphore()->take_blocking();
    if (!_dev->transfer(nullptr, 0, &portBits, sizeof(portBits))) {
        _dev->get_semaphore()->give();
        return false;
    }
    _dev->get_semaphore()->give();

    //Search for the first set bit, then return its location
    for (uint8_t x = 0; x < 8; x++)
    {
        if (portBits & (1 << x)) return (x);
    }
    
    return (255); //Return no port set
}

//Writes a 8-bit value to mux
//Overwrites any other bits
//This allows us to enable/disable multiple ports at same time
bool QWIICMUX::setPortState(uint8_t portBits)
{
    _dev->get_semaphore()->take_blocking();
    if (!_dev->transfer(&portBits, sizeof(portBits), nullptr, 0)) {
        _dev->get_semaphore()->give();
        return false; //Device did not ACK
    }
    _dev->get_semaphore()->give();

    return true;
}

uint8_t QWIICMUX::getPortState()
{
    uint8_t portBits;
    //Read the current mux settings (in Bits)
    _dev->get_semaphore()->take_blocking();
    if (!_dev->transfer(nullptr, 0, &portBits, sizeof(portBits))) {
        _dev->get_semaphore()->give();
        return 0;
    }
    _dev->get_semaphore()->give();

    return portBits;
}

bool QWIICMUX::enablePort(uint8_t portNumber)
{
    if (portNumber > 7) portNumber = 7; //Error check

    //Read the current mux settings
    uint8_t settings = getPortState();

    //Set the wanted bit to enable the port
    settings |= (1 << portNumber);

    return (setPortState(settings));
}

//Disables a specific port number
bool QWIICMUX::disablePort(uint8_t portNumber)
{
    if (portNumber > 7) portNumber = 7; //Error check

    uint8_t settings = getPortState();

    //Clear the wanted bit to disable the port
    settings &= ~(1 << portNumber);

    return (setPortState(settings));
}