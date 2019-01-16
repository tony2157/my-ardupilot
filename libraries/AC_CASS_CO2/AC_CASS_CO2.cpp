/**
 * CO2 Sensor Class, implemented for the SenseAir K30FR sensor with I2C.
 */
#include "AC_CASS_CO2.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h> 
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;
//AP_HAL::UARTDriver* myUART;
const uint8_t readCMD[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
uint16_t resp[] = {0,0,0,0,0,0,0};  //create an array to store the response
const int16_t BUFF_LENGTH = 7;
//const uint8_t DELAY_LIMIT = 20;
const int NEW_CYCLE_LIMIT = 4;

AC_CASS_CO2::AC_CASS_CO2() :
    _dev(nullptr),
    _co2Value(42),
    _errorCode(0),
    _healthy(false),
    //_callCount(0),
    _isRead(false)   
{
}

bool AC_CASS_CO2::initUART0(){
    _myUARTId = 0;
    return _initUART(hal.uartE);
    return true;
}

bool AC_CASS_CO2::initUART1(){
    _myUARTId = 1;
    return _initUART(hal.uartD);
}

/*
* Telem 1 is Serial 1
* Telem 2 is serial 2
* GPS 1 is serial 3
* GPS 2 is Serial 4
* Debug is serial 5
* uartA - the console (usually USB, runs MAVLink telemetry) = Serial_0
* uartB - the first GPS = Serial_3
* uartC - primary telemetry (telem1 on Pixhawk, 2nd radio on APM2) = Serial_1
* uartD - secondary telemetry (telem2 on Pixhawk) = Serial_2
* uartE - 2nd GPS = Serial_4
*/
bool AC_CASS_CO2::_initUART(AP_HAL::UARTDriver* myUART){
    hal.scheduler->delay(1000);
    myUART->begin(9600, 7, 7);
    //hal.scheduler->register_delay_callback(_timer, (uint16_t) 500);
    //myUART->register_periodic_callback(500000 FUNCTOR_BIND_MEMBER(&AC_CASS_CO2::_timer, void));
    return true;
}

 bool AC_CASS_CO2::readCO2(){
     if(_myUARTId == 0){
        return internalRead(hal.uartE);
     }else{
        return internalRead(hal.uartD);
     }
     
 }

 bool AC_CASS_CO2::internalRead(AP_HAL::UARTDriver* myUART){
    if(_isRead){
        int i;
        int16_t nbytes = myUART->available();
        if(nbytes < BUFF_LENGTH){
            _co2Value = 0;
            _errorCode = 700;
        }else{
            for (i=0; i<BUFF_LENGTH; i++) {
                    resp[i] = myUART->read();  
            }
            //high byte for value is 4th byte in packet in the packet
            int16_t high = resp[3]; 
            //low byte for value is 5th byte in the packet
            int16_t low = resp[4];
            _co2Value = high*256 + low;
            _errorCode = 0;
        }
        _isRead = false;
    }else{
        if(myUART->is_initialized() == 0){
            _errorCode = 600;    
        }else{
            myUART->write(readCMD, BUFF_LENGTH);
            _errorCode = 0;
        }
        _isRead = true;
    }


    /*
    int i;
    int16_t nbytes = myUART->available();
    if(nbytes < BUFF_LENGTH){
        _co2Value = 0;
        _errorCode = 320;
    }else{
        for (i=0; i<BUFF_LENGTH; i++) {
                resp[i] = myUART->read();  
        }
        //high byte for value is 4th byte in packet in the packet
        int16_t high = resp[3]; 
        //low byte for value is 5th byte in the packet
        int16_t low = resp[4];
        _co2Value = high*256 + low;
        _errorCode = 0;
    }

    myUART->write(readCMD, BUFF_LENGTH);
    

    //int j = 0;

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            hal.console->printf("_callCount: %d\n", _callCount);
            hal.console->printf("_wasRead: %s\n", (_wasRead)?"true":"false");
    #endif

    if(_callCount == 0){
        //New Cycle  
        myUART->write(readCMD, BUFF_LENGTH);
        _wasRead = false;
        for (i=0; i<BUFF_LENGTH; i++) {
            resp[i] = 0;
        }
        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            hal.console->printf("WRITE\n");
        #endif
    }
    
    if(_wasRead == false){
        if(myUART->available() >= 7){
            _wasRead = true;
            for (i=0; i<BUFF_LENGTH; i++) {
                resp[i] = myUART->read();  
            }
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                hal.console->printf("READ\n");
            #endif
            //high byte for value is 4th byte in packet in the packet
            int16_t high = resp[3]; 
            //low byte for value is 5th byte in the packet
            int16_t low = resp[4];
            _co2Value = high*256 + low;
        }
    }

    if (_wasRead == true){
           _co2Value = 1024;
           _errorCode = 1000;        
    }
    
    _callCount++;
    if(_callCount > NEW_CYCLE_LIMIT){
       _callCount = 0;
       if (_wasRead == false){
           //_co2Value = 0;
           //_errorCode = 300;        
        }
    }


    while((myUART->available() < BUFF_LENGTH) && (j<DELAY_LIMIT)){
        hal.scheduler->delay(10);
        j++;
    }
    if(j>=DELAY_LIMIT && (myUART->available() < BUFF_LENGTH)){
        _errorCode = 300;
        return false;
    }
    int i;
    for (i=0; i<BUFF_LENGTH; i++) {
        resp[i] = myUART->read();
    }  */  

    
    return true;
}

/////////////////////////////////////////////OLD//////////////////////////////////////////////////////////////

//bool AC_CASS_CO2::init()
bool AC_CASS_CO2::init(uint8_t busId, uint8_t i2cAddr){
   sem = hal.util->new_semaphore();

    // Zero is the number of the I2C Bus of the PIXHAWK2.
    // One is the number of the I2C Bus of the PIXHAWK1 or GPS1 Bus on PIXHAWK2.
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    //_dev->set_split_transfers(true);
    if (!_dev) {
        printf("CO2 device is null!");
        _co2Value = 0;
        _errorCode = 100;
        return false;
    }

    if (!_dev->get_semaphore()->take(0)) {
        AP_HAL::panic("PANIC: C02: failed to take serial semaphore for init");
    }

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    _dev->set_retries(10);

    if (!_request()) {
        printf("CO2 read failed");
        _dev->get_semaphore()->give();
        _errorCode = 200;
        _co2Value = 0;
        return false;
    }

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    // Request 2Hz update
    // Max conversion time is 10 ms
    _dev->register_periodic_callback(500000,
                                     FUNCTOR_BIND_MEMBER(&AC_CASS_CO2::_timer, void));                             
    return true;
}

void AC_CASS_CO2::set_i2c_addr(uint8_t addr){
    if (_dev) {
        _dev->set_address(addr);
    }
}

/*
 * Function that askes the sensor for the data, that will be collected in a future call.
 */ 
bool AC_CASS_CO2::_request(){
    uint8_t cmd[4] = {0x22, 0x00, 0x08, 0x2A};
 
    if (!_dev->transfer(cmd, sizeof(cmd), nullptr, 0)) {
        return false;
    }
    return true;        
}  

/*
 * Function that retrieves the data from the sensor.
 */ 
bool AC_CASS_CO2::_collect(){
    uint8_t checkSum;
    uint8_t data[4];
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    
    //Read sensors
    if (!_dev->transfer(nullptr, 0, data, sizeof(data))) {
        return false;
    }

    _co2Value = 0;
    _errorCode = 0;    
    checkSum = 0x00;

    _co2Value |= data[1] & 0xFF;
    _co2Value = _co2Value << 8;
    _co2Value |= data[2] & 0xFF;
    checkSum = (data[0] + data[1] + data[2]); //addition utilizes overflow

    if (data[3] != checkSum){
        _errorCode = 300; 
        _co2Value = 0;
        return false;
    }

    
    return true;  
} 

/*
 * Function that is callback by the processor at a 2Hz intervall.
 */ 
void AC_CASS_CO2::_timer(void){  
   readCO2();
    /*if(sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)){
     _healthy = _request();
     sem->give();
    } else{
        _co2Value = 0;
        _errorCode = 500;
        return;
    }
    hal.scheduler->delay(11000);
    if(sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)){    
        if (_healthy) {
            _collect();
        } else {
            //error!
            _co2Value = 0;
            //_errorCode = 400;
        }
        //_healthy = _request();
        sem->give();
    }  else{
        _co2Value = 0;
        _errorCode = 600;
    } */
}
