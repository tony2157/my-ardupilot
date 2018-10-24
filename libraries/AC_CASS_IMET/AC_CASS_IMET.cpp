#include "AC_CASS_Imet.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AC_CASS_Imet::AC_CASS_Imet() :
    _dev(nullptr),
    _temperature(0),
    _volt(0),
    _healthy(false)
{
}

bool AC_CASS_Imet::init()
{

    flag = false;
    adc_curr = 0;
    adc_volt = 0;
    sem = hal.util->new_semaphore();

    for(uint8_t i=0; i<3; i++){
        memset(coeff,1.0,sizeof(coeff));
    }
    
    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
             ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
             ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
             ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
             ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
             ADS1015_REG_CONFIG_MODE_SINGLE  | // Single-shot mode (default)
             ADS1015_REG_CONFIG_PGA_6_144V   | // Set PGA/voltage range
             ADS1015_REG_CONFIG_OS_SINGLE;     // Set start single-conversion bit

    // Bus 0 is for Pixhawk 2.1 I2C and Bus 1 is for Pixhawk 1 and PixRacer I2C
    _dev = std::move(hal.i2c_mgr->get_device(0, IMET_DEFAULT_ADDR));
    if (!_dev) {
        printf("IMET device is null!");
        return false;
    }

    if (!_dev->get_semaphore()->take(0)) {
        AP_HAL::panic("PANIC: IMET: failed to take serial semaphore for init");
    }

    _dev->set_retries(10);

    if (!_config_read_volt()) {
        printf("IMET read failed");
        _dev->get_semaphore()->give();
        return false;
    }

    hal.scheduler->delay(5);

    if (!_config_read_curr()) {
        printf("IMET read failed");
        _dev->get_semaphore()->give();
        return false;
    }

    hal.scheduler->delay(20);

    _read_adc();

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    /* Request 25Hz update */
    // Max conversion time is 12 ms
    _dev->register_periodic_callback(40000,
                                     FUNCTOR_BIND_MEMBER(&AC_CASS_Imet::_timer, void));
    return true;
}

void AC_CASS_Imet::set_i2c_addr(uint8_t addr){
    if (_dev) {
        _dev->set_address(addr);
    }
}

void AC_CASS_Imet::set_sensor_coeff(float *k){
    for(uint8_t i=0; i<3; i++){
        coeff[i] = k[i];
    }
}

bool AC_CASS_Imet::_config_read_curr()
{
    struct PACKED {
            uint8_t reg;
            be16_t val;
        } config_pack;

        config_pack.reg = 0x01;
        config_pack.val = htobe16(config | ADS1015_REG_CONFIG_MUX_SINGLE_0);

        if (!_dev->transfer((uint8_t *)&config_pack, sizeof(config_pack), nullptr, 0)) {
            return false;
        }

        return true;
}

bool AC_CASS_Imet::_config_read_volt()
{
    struct PACKED {
            uint8_t reg;
            be16_t val;
        } config_pack;

        config_pack.reg = 0x01;
        config_pack.val = htobe16(config | ADS1015_REG_CONFIG_MUX_SINGLE_1);

        if (!_dev->transfer((uint8_t *)&config_pack, sizeof(config_pack), nullptr, 0)) {
            return false;
        }

        return true;
}

float AC_CASS_Imet::_read_adc()
{
    uint8_t status[2];
    uint8_t data[2];

        if (!_dev->read_registers(0x01, status, sizeof(status))) {
            return 0;
        }

        /* check rdy bit */
        if ((status[1] & 0x80) != 0x80 ) {
            return 0;
        }

        // An easier way of reading registers
        if (!_dev->read_registers(0x00, data, sizeof(data))) {
            return 0;
        }
        return (float)((data[0] << 8) | data[1]);
}

void AC_CASS_Imet::_timer(void)
{
    if(flag == false){
        adc_curr = _read_adc();
        _config_read_volt();
    }
    else{
        adc_volt = _read_adc();
        _config_read_curr();
    }

    flag = !flag;

    if(sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)){
        if(adc_curr == 0.0 || adc_volt == 0.0){
            _healthy = false;
        }
        else{
            _healthy = true;
        }

        if (_healthy) {
            _calculate(adc_volt, adc_curr);
        } else {
            _temperature = 0;
        }
        sem->give();
    }
}

void AC_CASS_Imet::_calculate(float volt, float curr)
{
    float resist = 64900.0f * (volt / curr - 1.0f);
    _volt = curr * 0.1875f; //to store the voltage and send it
    //converts to temperature (kelvin)
    _temperature = 1.0f / (coeff[0] + coeff[1] * (float)log(resist) + coeff[2] * (float)pow((float)log((float)resist), 3));
}