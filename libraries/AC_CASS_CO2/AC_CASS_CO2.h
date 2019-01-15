/*
 * Header file for the CO2 Sensor Class.
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>

/* Address used if none is provided at sensor initialization. */
#define CO2_DEFAULT_ADDR 0x68

class AC_CASS_CO2 {
public:
    AC_CASS_CO2(void);
    ~AC_CASS_CO2(void){};

  // bool init();
    bool init(uint8_t busId, uint8_t i2cAddr); //Identifer for which I2C should the device be registered. I2C addr for the sensor being initiated.
    bool initUART0();
    bool initUART1();
    bool readCO2();
    uint16_t co2Value(void) { return _co2Value;} // CO2 value in ppm.
    uint16_t errorCode(void) { return _errorCode;} // CO2 value in ppm.
    bool healthy(void) { return _healthy;} // Indicates if there is a valid CO2 Measurement.
    void set_i2c_addr(uint8_t addr); // Defines the i2c addr of the instance.

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    AP_HAL::Semaphore *sem;
    uint16_t _co2Value; // Value in ppm, range: 0-5000, accuracy +/- 30ppm.
    uint16_t _errorCode; // 0 - no error; 100 -  device is null!; 200 - CO2 read failed; 300 - checksum.
    uint8_t _myUARTId; 
    bool _isRead; //Indicates if a response was read. 0 = NO, 1 = YES;
    bool _healthy; // True if there is a valid co2Value reading available.
    bool _request(void); // True if a measurement request was delivered to the sensor.
    bool _collect(); // True if a measurement value was acquired from sensor.
    void _timer(void); // updates the co2Value, called at 2Hz.
    //bool _initUART(uint8_t portId);
    bool _initUART(AP_HAL::UARTDriver* myUART);
    bool internalRead(AP_HAL::UARTDriver* myUART);
};