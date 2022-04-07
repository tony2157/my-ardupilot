#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define SENSOR_I2C_BASE_ADDRESS		0x4C
#define SENSOR_TIMEOUT_MAX		0x6000

class AP_ARRC_LB5900 {
public:
    AP_ARRC_LB5900(void);
    ~AP_ARRC_LB5900(void){}

    bool init(uint8_t busId, uint8_t i2cAddr, uint16_t freq, uint8_t avg_cnt);
    float power_measure(void) { return _power; } // temperature in kelvin
    bool healthy(void) { return _healthy; } // do we have a valid temperature reading?
    void set_i2c_addr(uint8_t addr);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    HAL_Semaphore _sem; // semaphore for access to shared frontend data
    float _power; //voltage read by the ADC
    bool _ready; // we have a valid temperature reading to report
    uint32_t Sensor_TimeOut; // Value of Timeout when I2C communication fails

    static unsigned int commandNumber;

    union{
        uint8_t byte[5000];
        struct
        {
            uint8_t commandAndLength[4];
            uint8_t buffer[4096];
        }field;
    }write_sensor_buffer;

    union{
        uint8_t byte[5000];
        struct
        {
            uint8_t statusAndLength[4];
            uint8_t buffer[4096];
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

    bool configSensor(uint16_t freq, uint8_t avg_cnt);
    bool _measure(void);
    bool _status_length(void);
    bool _collect(float &_power);
    void _timer(void); // update the temperature, called at 20Hz
};