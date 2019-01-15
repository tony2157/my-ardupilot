#include "Plane.h"
#include <utility>
#include <SRV_Channel/SRV_Channel.h>

//Humidity sensor Params
const int N_RH = 4;     //supports up to 4
float raw_H[4], rawRHt[4];
uint16_t RHerror[2];

//IMET sensor Params
const int N_imet = 4;   //supports up to 4
float volt[4], curr[4];

//CO2 sensor Params
uint16_t co2Val[2]; // One integer value for each sensor.
uint16_t co2Error[2]; // One integer value for each sensor.

#ifdef USERHOOK_INIT
void Plane::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //Initialize Global Variables
    memset(raw_H,0,sizeof(raw_H));
    memset(rawRHt,0,sizeof(rawRHt));
    memset(volt,0,sizeof(volt));
    memset(curr,0,sizeof(curr));
}
#endif

#ifdef USERHOOK_CO2LOOP
void Plane::readCO2_0(){
    plane.CASS_CO2[0].readCO2();
}

void Plane::readCO2_1(){
    plane.CASS_CO2[1].readCO2(); 
}

void Plane::userhook_CO2(){   
    co2Val[0] = plane.CASS_CO2[0].co2Value();
    co2Error[0] = plane.CASS_CO2[0].errorCode();
    co2Val[1] = plane.CASS_CO2[1].co2Value();
    co2Error[1] = plane.CASS_CO2[1].errorCode();
    
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        uint32_t tStamp = AP_HAL::millis();
        co2Val[0] = 570 + sin(tStamp) * 25;
        co2Val[1] = sin(tStamp) * 25;
    #endif

    struct log_CO2 pkt_CO2 = {
        LOG_PACKET_HEADER_INIT(LOG_CO2_MSG),
        time_stamp : AP_HAL::micros64(),
        co2Value0 : co2Val[0],
        errorCode0 : co2Error[0], 
        co2Value1 : co2Val[1],
        errorCode1 :co2Error[1]
    };

    plane.DataFlash.WriteBlock(&pkt_CO2, sizeof(pkt_CO2));

    //Send data to ground station
    //plane.send_cass_data(1, co2Val, 2);
}
#endif

#ifdef USERHOOK_TEMPLOOP
///////////// READ TEMPERATURE SENSORS FUNCTION /////////////////////////////////////////////////
void Plane::userhook_TempLoop()
{
   // Read temperature

    curr[0] = plane.CASS_Imet[0].temperature();

    curr[1] = plane.CASS_Imet[1].temperature();

    curr[2] = plane.CASS_Imet[2].temperature();

    curr[3] = plane.CASS_Imet[3].temperature();

    // Read voltage

    volt[0] = plane.CASS_Imet[0].voltage();

    volt[1] = plane.CASS_Imet[1].voltage();

    volt[2] = plane.CASS_Imet[2].voltage();

    volt[3] = plane.CASS_Imet[3].voltage();



    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL

        uint32_t m = AP_HAL::millis();

        curr[0] = 298.15 + sin(m) * 25;

        curr[1] = 1 + curr[0];

        curr[2] = 1 + curr[1];

        curr[3] = 1 + curr[2];

        // curr[0] = 1;

        // curr[1] = 2;

        // curr[2] = 3;

        // curr[3] = 4;        

    #endif



    // Write sensors packet into the SD card

    struct log_IMET pkt_temp = {

        LOG_PACKET_HEADER_INIT(LOG_IMET_MSG),

        time_stamp             : AP_HAL::micros64(),

        temperature1           : curr[0],

        voltage1               : volt[0],

        temperature2           : curr[1],

        voltage2               : volt[1],

        temperature3           : curr[2],

        voltage3               : volt[2],

        temperature4           : curr[3],

        voltage4               : volt[3]

    };

    plane.DataFlash.WriteBlock(&pkt_temp, sizeof(pkt_temp));



    // Send data to ground station

    //plane.send_cass_data(0, curr, 4);



    // cliSerial->printf("\ncurr3: %f", curr[3]);
}
#endif

#ifdef USERHOOK_RHLOOP

///////////// READ RH SENSORS FUNCTION /////////////////////////////////////////////////
void Plane::userhook_RHLoop(){
  // Read Realative Humidity

    raw_H[0] = plane.CASS_HYT271[0].relative_humidity();

    raw_H[1] = plane.CASS_HYT271[1].relative_humidity();

    raw_H[2] = plane.CASS_HYT271[2].relative_humidity();

    raw_H[3] = plane.CASS_HYT271[3].relative_humidity();



    // Read Temperature

    rawRHt[0] = plane.CASS_HYT271[0].temperature();

    rawRHt[1] = plane.CASS_HYT271[1].temperature();

    rawRHt[2] = plane.CASS_HYT271[2].temperature(); 

    rawRHt[3] = plane.CASS_HYT271[3].temperature(); 

         

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL

        uint32_t m = AP_HAL::millis();

        rawRHt[0] = 298.15 + sin(m) * 25;

        rawRHt[1] = 1 + rawRHt[0];

        rawRHt[2] = 1 + rawRHt[1];

        rawRHt[3] = 1 + rawRHt[2];



        raw_H[0] = abs(sin(m) * 95);

        raw_H[1] = 1 + raw_H[0];

        raw_H[2] = 1 + raw_H[1];

        raw_H[3] = 1 + raw_H[2];

    #endif   


    // Write sensors packet into the SD card

    struct log_RH pkt_RH = {

        LOG_PACKET_HEADER_INIT(LOG_RH_MSG),

        time_stamp             : AP_HAL::micros64(), //- _last_read_ms),

        humidity1              : raw_H[0],

        RHtemp1                : rawRHt[0],

        humidity2              : raw_H[1],

        RHtemp2                : rawRHt[1],

        humidity3              : raw_H[2],

        RHtemp3                : rawRHt[2],

        humidity4              : raw_H[3],

        RHtemp4                : rawRHt[3]

    };


    plane.DataFlash.WriteBlock(&pkt_RH, sizeof(pkt_RH));

    //Send data to ground station

    //plane.send_cass_data(1, raw_H, 4);

    //plane.send_cass_data(2, rawRHt, 4);  
}
#endif