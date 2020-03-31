#include "Copter.h"
#include <utility>
#include <SRV_Channel/SRV_Channel.h>
//#include <Filter/LowPassFilter2p.h>
#include <Filter/cheby2LPF.h>

//Humidity sensor Params
const int N_RH = 4;     //supports up to 4

//IMET sensor Params
const int N_imet = 4;   //supports up to 4

//Fan control params
//It will run the fan at SERVO_MAX, which can be set in the params list
uint16_t fan_pwm_on = 100; // 40
uint16_t fan_pwm_off = 0;
bool _fan_status;

//Wind estimator Params
float wsA = 32.8;               //Coefficient A of the linear wind speed equation, from calibration
float wsB = -4.5;               //Coefficient B of the linear wind speed equation, from calibration
float _wind_speed, _wind_dir;
float R13, R23;
float last_yrate;
float min_roll = 0.5;            //Minimum roll angle that the wind vane will correct (too low and the copter will oscilate)
float vane_rate = 1;      //Maximum yaw angle rate at which the Copter will rotate
float vane_gain = 1;     //Wind vane gain: higher values will increase the resposivness

//Declare digital LPF
cheby2LPFFloat wind_x_filter;
cheby2LPFFloat wind_y_filter;
// LowPassFilter2pFloat wind_x_filter;
// LowPassFilter2pFloat wind_y_filter;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //Initialize Wind estimator
    _wind_dir = 0.0f;  _wind_speed = 0.0f;
    R13 = 0.0f; R23 = 0.0f;
    last_yrate = 0;

    //Wind filter initialization
    wind_x_filter.set_cutoff_frequency(20,0.06);
    wind_y_filter.set_cutoff_frequency(20,0.06);

    // Initialize Fan Control
    SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
    _fan_status = false;
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
    // put your 10Hz code here
    #if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    // Read Temperature, Resistance and Health. Write sensors packet into the SD card
    struct log_IMET pkt_temp = {
        LOG_PACKET_HEADER_INIT(LOG_IMET_MSG),
        time_stamp             : AP_HAL::micros64(),                //Store time in microseconds
        fan_status             : _fan_status,                       //Store Fan on/off status
        healthy1               : copter.CASS_Imet[0].healthy(),     //Store sensor health
        healthy2               : copter.CASS_Imet[1].healthy(),
        healthy3               : copter.CASS_Imet[2].healthy(),
        healthy4               : copter.CASS_Imet[3].healthy(),
        temperature1           : copter.CASS_Imet[0].temperature(), //Store iMet Temperature
        temperature2           : copter.CASS_Imet[1].temperature(),
        temperature3           : copter.CASS_Imet[2].temperature(),
        temperature4           : copter.CASS_Imet[3].temperature(),
        resist1                : copter.CASS_Imet[0].resistance(),  //Store iMet bead resistence
        resist2                : copter.CASS_Imet[1].resistance(),
        resist3                : copter.CASS_Imet[2].resistance(),
        resist4                : copter.CASS_Imet[3].resistance()
    }; 
    #else
        uint32_t m = AP_HAL::millis();
        float alt; float simT;
        copter.ahrs.get_relative_position_D_home(alt);
        alt = -1*alt;  
        if(alt < 50){
            simT = 300 - 2*alt*alt/2500;
        }
        else if(alt < 150){
            simT = 298 - 0.01*alt;
        }
        else if(alt < 200){
            simT = 296 + 2*alt*alt/40000;
        }
        else{
            simT = 298;
        }
        // Write simulated sensors packet into the SD card
        struct log_IMET pkt_temp = {
            LOG_PACKET_HEADER_INIT(LOG_IMET_MSG),
            time_stamp             : AP_HAL::micros64(),
            fan_status             : _fan_status,
            healthy1               : copter.CASS_Imet[0].healthy(),
            healthy2               : copter.CASS_Imet[1].healthy(),
            healthy3               : copter.CASS_Imet[2].healthy(),
            healthy4               : copter.CASS_Imet[3].healthy(),
            temperature1           : simT + sinf(0.0003f*m) * 0.001f,
            temperature2           : simT + sinf(0.0004f*m) * 0.001f,
            temperature3           : simT + sinf(0.0005f*m) * 0.001f,
            temperature4           : simT + sinf(0.0006f*m) * 0.001f,
            resist1                : copter.CASS_Imet[0].resistance(),
            resist2                : copter.CASS_Imet[1].resistance(),
            resist3                : copter.CASS_Imet[2].resistance(),
            resist4                : copter.CASS_Imet[3].resistance()
        };
    #endif
    copter.DataFlash.WriteBlock(&pkt_temp, sizeof(pkt_temp));   //Send package to SD card
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
     #if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        // Read Rel. Humidity, Temperature and Health. Write sensors packet into the SD card
        struct log_RH pkt_RH = {
            LOG_PACKET_HEADER_INIT(LOG_RH_MSG),
            time_stamp             : AP_HAL::micros64(),                        //Store time in microseconds
            healthy1               : copter.CASS_HYT271[0].healthy(),           //Store senors health
            healthy2               : copter.CASS_HYT271[1].healthy(),
            healthy3               : copter.CASS_HYT271[2].healthy(),
            healthy4               : copter.CASS_HYT271[3].healthy(),
            humidity1              : copter.CASS_HYT271[0].relative_humidity(), //Store Rel. humidity
            humidity2              : copter.CASS_HYT271[1].relative_humidity(),
            humidity3              : copter.CASS_HYT271[2].relative_humidity(),
            humidity4              : copter.CASS_HYT271[3].relative_humidity(),
            RHtemp1                : copter.CASS_HYT271[0].temperature(),       //Store temperature
            RHtemp2                : copter.CASS_HYT271[1].temperature(),
            RHtemp3                : copter.CASS_HYT271[2].temperature(),
            RHtemp4                : copter.CASS_HYT271[3].temperature()
        };
    #else
        uint32_t m = AP_HAL::millis();
        // Write sensors packet into the SD card
        struct log_RH pkt_RH = {
            LOG_PACKET_HEADER_INIT(LOG_RH_MSG),
            time_stamp             : AP_HAL::micros64(),
            healthy1               : copter.CASS_HYT271[0].healthy(),
            healthy2               : copter.CASS_HYT271[1].healthy(),
            healthy3               : copter.CASS_HYT271[2].healthy(),
            healthy4               : copter.CASS_HYT271[3].healthy(),
            humidity1              : (float)fabs(sinf(0.0003f*m) * 60.0f),
            humidity2              : (float)fabs(sinf(0.0004f*m) * 60.0f),
            humidity3              : (float)fabs(sinf(0.0005f*m) * 60.0f),
            humidity4              : (float)fabs(sinf(0.0006f*m) * 60.0f),
            RHtemp1                : 298.15f + sinf(0.0003f*m) * 2.0f,
            RHtemp2                : 298.15f + sinf(0.0004f*m) * 2.0f,
            RHtemp3                : 298.15f + sinf(0.0005f*m) * 2.0f,
            RHtemp4                : 298.15f + sinf(0.0006f*m) * 2.0f
        };
    #endif
    copter.DataFlash.WriteBlock(&pkt_RH, sizeof(pkt_RH));   //Send package to SD card
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    //Run Algo after Copter takes off
    if(!ap.land_complete && copter.position_ok()){ // !arming.is_armed(), !ap.land_complete, motors->armed()

        float alt;
        copter.ahrs.get_relative_position_D_home(alt);
        alt = -100.0f*alt;           // get AGL altitude in cm

        //Fan Control    
        if(alt > 185.0f && SRV_Channels::get_output_scaled(SRV_Channel::k_egg_drop) < 50){
            SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_on);
            _fan_status = true;
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                printf("FAN ON \n");  //printf("PWM: %5.2f \n",var); //for debugging
            #endif
        }
        else{
            if(alt < 140.0f && SRV_Channels::get_output_scaled(SRV_Channel::k_egg_drop) > 50){
                SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
                _fan_status = false;
                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    printf("FAN OFF \n");  
                #endif
            }
        }

        //Get current velocity and horizontal distance to next waypoint
        Vector3f vel_xyz = copter.inertial_nav.get_velocity();
        float speed_xy = norm(vel_xyz.x,vel_xyz.y); // cm/s
        float dist_to_wp = copter.wp_nav->get_wp_distance_to_destination(); // cm (horizontally)

        //Wind vane is active when flying horizontally steady
        if(speed_xy < 120.0f && dist_to_wp < 500){
            //Wind Estimator Algorithm
            //Get current target attitude from the attitude controller
            float troll = copter.wp_nav->get_roll()/100.0f;
            float tpitch = copter.wp_nav->get_pitch()/100.0f;

            //Get thurst vector elements from the rotation matrix
            R13 = -1*copter.ahrs.get_rotation_body_to_ned().a.z;
            R23 = -1*copter.ahrs.get_rotation_body_to_ned().b.z;

            //Filter and determine wind direction by trigonometry (thrust vector tilt)
            float wind_x = wind_x_filter.apply(R13); //2nd order LPF, 20Hz sampling, 0.1Hz cutoff
            float wind_y = wind_y_filter.apply(R23); //2nd order LPF, 20Hz sampling, 0.1Hz cutoff
            float wind_psi = fmodf(atan2f(wind_y,wind_x),2*M_PI)*180.0f/M_PI;

            //Define a dead zone around zero roll
            if(fabsf(troll) < min_roll){ last_yrate = 0; }

            //Convert roll magnitude into desired yaw rate
            float yrate = constrain_float((troll/5.0f)*vane_gain,-vane_rate,vane_rate);
            last_yrate = 0.98f*last_yrate + 0.02f*yrate; //1st order LPF

            //For large compensation use "wind_psi" estimator, for fine adjusments use "yrate" estimator
            if(fabsf(troll)<0){
                _wind_dir = copter.cass_wind_direction/100.0f + last_yrate;
                _wind_dir = wrap_360_cd(_wind_dir*100.0f);
            }
            else{ 
                _wind_dir = wrap_360_cd(wind_psi*100.0f);
                last_yrate = 0;
            }

            //Estimate wind speed
            _wind_speed = wsA * sqrtf(tanf(fabsf(tpitch)*M_PI/180.0f)) + wsB;

            //Min altitude at which the yaw command is sent
            if(alt>400.0f){
                //Send wind direction to the autopilot
                copter.cass_wind_direction = _wind_dir;
                copter.cass_wind_speed = _wind_speed;
            }
            else{
                copter.cass_wind_direction = copter.wp_nav->get_yaw();
                copter.cass_wind_speed = 0.0f;
            }
        }
        else{
            wind_x_filter.reset();
            wind_y_filter.reset();
            last_yrate = 0.0f;
        }
    }
    else{
        copter.cass_wind_direction = (float)copter.initial_armed_bearing;
        copter.cass_wind_speed = 0.0f;
        SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
        wind_x_filter.reset();
        wind_y_filter.reset();
        last_yrate = 0;
        _fan_status = false;
    }

    // Write wind direction packet into the SD card
    struct log_WIND pkt_wind_est = {
        LOG_PACKET_HEADER_INIT(LOG_WIND_MSG),
        time_stamp             : AP_HAL::micros64(),// - _last_read_ms),
        _wind_dir              : _wind_dir/100,
        _wind_speed            : _wind_speed,
        _R13                   : R13,
        _R23                   : R23
    };
    copter.DataFlash.WriteBlock(&pkt_wind_est, sizeof(pkt_wind_est));

    // float data[5] = {0};
    // data[0] = _wind_dir/100.0f;
    // data[1] = _wind_speed;
    // data[2] = Vel_xy;
    // data[3] = var_wind_dir;
    // data[4] = alt;
    // copter.send_cass_data(3, data, 5);
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
