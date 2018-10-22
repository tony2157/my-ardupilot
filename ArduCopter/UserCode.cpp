#include "Copter.h"
#include <utility>
#include <SRV_Channel/SRV_Channel.h>

//Humidity sensor Params
const int N_RH = 4;     //supports up to 4
float raw_H[4], rawRHt[4];

//IMET sensor Params
const int N_imet = 4;   //supports up to 4
float volt[4], curr[4];

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
        // put your initialisation code here
    // this will be called once at start-up

    //Initialize Global Variables
    memset(raw_H,0,sizeof(raw_H));
    memset(rawRHt,0,sizeof(rawRHt));
    memset(volt,0,sizeof(volt));
    memset(curr,0,sizeof(curr));

    //Initialize Wind estimator
    _wind_dir = 0.0f; _roll = 0.0f; _pitch = 0.0f; _yaw = 0.0f;
    A0 = 0.011f; //m2
    Cd = 4.8f;
    rho = 1.225f; //kg/m3
    mass = 22.6f; //N
    Ps = 0.1f;
    R = 0.14f; //m
    _roll_sum = 0.0f; _pitch_sum = 0.0f;
    avgR = 0.0; avgP = 0.0;
    var_temp_dir = 0.0f; var_temp_gamma = 0.0f;
    _wind_dir = 0.0f;  _wind_speed = 0.0f;
    k = 0;
    memset(mean_aux_dir,0.0f,sizeof(mean_aux_dir));
    memset(var_aux_dir,1000.0f,sizeof(var_aux_dir));
    memset(mean_aux_speed,0.0f,sizeof(mean_aux_speed));
    memset(var_aux_speed,1000.0f,sizeof(var_aux_speed));
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // Read temperature
    curr[0] = copter.CASS_Imet[0].temperature();
    curr[1] = copter.CASS_Imet[1].temperature();
    curr[2] = copter.CASS_Imet[2].temperature();
    curr[3] = copter.CASS_Imet[3].temperature();
    // Read voltage
    volt[0] = copter.CASS_Imet[0].voltage();
    volt[1] = copter.CASS_Imet[1].voltage();
    volt[2] = copter.CASS_Imet[2].voltage();
    volt[3] = copter.CASS_Imet[3].voltage();

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
    copter.DataFlash.WriteBlock(&pkt_temp, sizeof(pkt_temp));

    // Send data to ground station
    copter.send_cass_data(0, curr, 4);

    // cliSerial->printf("\ncurr3: %f", curr[3]);
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // Read Realative Humidity
    raw_H[0] = copter.CASS_HYT271[0].relative_humidity();
    raw_H[1] = copter.CASS_HYT271[1].relative_humidity();
    raw_H[2] = copter.CASS_HYT271[2].relative_humidity();
    raw_H[3] = copter.CASS_HYT271[3].relative_humidity();

    // Read Temperature
    rawRHt[0] = copter.CASS_HYT271[0].temperature();
    rawRHt[1] = copter.CASS_HYT271[1].temperature();
    rawRHt[2] = copter.CASS_HYT271[2].temperature(); 
    rawRHt[3] = copter.CASS_HYT271[3].temperature();   

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
    copter.DataFlash.WriteBlock(&pkt_RH, sizeof(pkt_RH));

    //Send data to ground station
    copter.send_cass_data(1, raw_H, 4);
    copter.send_cass_data(2, rawRHt, 4);  
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
