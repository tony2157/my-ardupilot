#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70

class QWIICMUX{
public:
    QWIICMUX(void);
    ~QWIICMUX(void){}

    bool init(uint8_t busId, uint8_t i2cAddr = QWIIC_MUX_DEFAULT_ADDRESS);
    bool setPort(uint8_t portNumber);   
    bool setPortState(uint8_t portBits);
    uint8_t getPort();
    uint8_t getPortState();
    bool enablePort(uint8_t portNumber);
    bool disablePort(uint8_t portNumber);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    void _timer(void); // update the temperature, called at 20Hz
};