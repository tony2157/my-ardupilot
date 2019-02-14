#include "Copter.h"
#include <utility>
#include <SRV_Channel/SRV_Channel.h>

//Humidity sensor Params
const int N_RH = 4;     //supports up to 4
float raw_H[4], rawRHt[4];

//IMET sensor Params
const int N_imet = 4;   //supports up to 4
float volt[4], curr[4];

//Fan control params
//Appartently, the range is now from 0 to 100%
uint16_t fan_pwm_on = 1210;
uint16_t fan_pwm_off = 800;
bool flag_fan_on = false;

//Wind estimator Params
const int N = 60;               //filter order
float _wind_speed, _wind_dir;   //Estimated parameters
float _roll, _pitch, _yaw;      //UAS attitude
float A0, Cd, rho, Drag;        //Estimator parameter: Minimum Area exposed, Drag coeff, air density, Drag force
float mass, Ps, R;              //UAS mass, Propeller Permeability coeff, prop radius
float var_temp_dir, var_temp_gamma;
float _roll_sum, _pitch_sum, avgR, avgP;
float mean_aux_dir[2],var_aux_dir[2];
float mean_aux_speed[2],var_aux_speed[2];
uint8_t k;                      //Number of measured wind dir/speed

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
    //copter.send_cass_data(0, curr, 4);

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
    //copter.send_cass_data(1, raw_H, 4);
    //copter.send_cass_data(2, rawRHt, 4);  
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    float var_gamma = 0.0f, var_wind_dir = 0.0f;
    float mean_gamma = 0.0f, mean_wind_dir = 0.0f;
    float speed = 0.0f, dist_to_wp = 0.0f;
    Vector3f e_angles, vel_xyz;
    float alt;

    copter.ahrs.get_relative_position_D_home(alt);
    alt = -1.0f*alt;

    //Fan Control    
    if(hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED){
        SRV_Channels::set_output_pwm(SRV_Channel::k_egg_drop, fan_pwm_off);
        flag_fan_on = false;
    }

    //Fan Control    
    if(hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED){
        SRV_Channels::set_output_pwm(SRV_Channel::k_egg_drop, fan_pwm_off);
        flag_fan_on = false;
    }
    else{
        if(alt > 2.5f){
            SRV_Channels::set_output_pwm(SRV_Channel::k_egg_drop, fan_pwm_on);
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                printf("FAN ON \n");  
            #endif
        }
        else{
            if(alt < 2.0f){
                SRV_Channels::set_output_pwm(SRV_Channel::k_egg_drop, fan_pwm_off);
                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    printf("FAN OFF \n");  
                #endif
            }
        }
    }

    //Start estimation after Copter took off
    if(!ap.land_complete){ // !arming.is_armed()
        // // FAN control. Turn on/off at given heights
        // if(alt > 1.8f && flag_fan_on == false){
        //     SRV_Channels::set_output_pwm(SRV_Channel::k_egg_drop, fan_pwm_on);
        // }
        // if(alt > 5.0f && flag_fan_on == false){
        //     flag_fan_on = true;
        // }
        // if(alt < 3.0f && flag_fan_on == true){
        //     SRV_Channels::set_output_pwm(SRV_Channel::k_egg_drop, fan_pwm_off);
        // }
        // if(alt < 1.0f && flag_fan_on == true){
        //     flag_fan_on == false;
        // }

        if(alt > 2.0f){
            float aux, A=0.0f; //Total area exposed to wind and aux variable

            //Current Attitude of the UAS
            copter.EKF2.getEulerAngles(-1,e_angles);
            _roll = e_angles.x;
            _pitch = e_angles.y;
            _yaw = e_angles.z;

            //Estimated horizontal velocity calculated by the EKF2
            copter.EKF2.getVelNED(-1,vel_xyz);
            speed = norm(vel_xyz.x,vel_xyz.y); // m/s
            dist_to_wp = copter.wp_nav->get_wp_distance_to_destination(); // cm (horizontally)

            if(speed < 1.5f && dist_to_wp < 800){
                if(k <= N-1){
                    // Roll and Pitch accumulation
                    _roll_sum = _roll_sum + _roll;
                    _pitch_sum = _pitch_sum + _pitch;
                    aux = atan2f(sinf(_roll), -sinf(_pitch));
                    var_temp_dir = var_temp_dir + aux*aux;
                    var_temp_gamma = var_temp_gamma + _pitch*_pitch;
                    k = k + 1;
                }
                else{
                    // 1st order Estimator, Mean and variance calculation
                    mean_gamma = avgP = _pitch_sum = _pitch_sum/N;
                    mean_wind_dir = avgR = _roll_sum = _roll_sum/N;
                    mean_wind_dir = atan2f(sinf(mean_wind_dir), -sinf(mean_gamma));
                    var_gamma = var_temp_gamma/N - mean_gamma*mean_gamma;
                    var_wind_dir = var_temp_dir/N - mean_wind_dir*mean_wind_dir;
                    if (var_wind_dir < 1e-6f){
                        var_wind_dir = 0.0f;
                    }
                    if (var_gamma < 1e-6f){
                        var_gamma = 0.0f;
                    }
                    var_gamma = sqrtf(var_gamma);
                    var_wind_dir = sqrtf(var_wind_dir);

                    //Filter wind speed measurements
                    if(var_gamma < 0.04f && alt > 4.0f){
                        if(fabs(_pitch_sum)>0.03){
                            _wind_speed = mass*fabs(tanf(mean_gamma));
                            A = A0 + 135.0f*Ps*R*R*fabs(sinf(mean_gamma));
                            _wind_speed = sqrtf(2.0f*_wind_speed/(rho*A*Cd));
                        }
                        else{
                            _wind_speed = 0;
                        }
                    }

                    //Filter wind direction measurements
                    if(var_wind_dir < 1.5f){
                        _wind_dir = wrap_360_cd((_yaw + mean_wind_dir)*5729.6f);
                        // Send wind direction to the Flight control 
                        if(alt>4.0f && var_wind_dir<0.8f && fabs(_pitch_sum)+fabs(_roll_sum)>0.05f){
                                copter.wp_nav->turn_into_wind_heading = _wind_dir;
                        }
                    }
                    // Singularities avoidance algorithm
                    else if (var_wind_dir >= 1.5f && alt > 4.0f){
                        if(fabs(_roll_sum) > fabs(_pitch_sum)){
                            if(_roll_sum > 0){
                                copter.wp_nav->turn_into_wind_heading = wrap_360_cd(copter.wp_nav->turn_into_wind_heading + 1000.0f);
                            }
                            else{
                                copter.wp_nav->turn_into_wind_heading = wrap_360_cd(copter.wp_nav->turn_into_wind_heading - 1000.0f);
                            }
                        }
                        else{
                            if(_pitch_sum > 0){
                                copter.wp_nav->turn_into_wind_heading = wrap_360_cd(copter.wp_nav->turn_into_wind_heading + 18000.0f);
                            }
                            else{
                                copter.wp_nav->turn_into_wind_heading = copter.wp_nav->turn_into_wind_heading;
                            }
                        }
                    }

                    //Reset variables for next loop
                    k = 0;
                    _roll_sum = 0.0f;
                    _pitch_sum = 0.0f;
                    var_temp_dir = 0.0f;
                    var_temp_gamma = 0.0f;
                }
            }
        }
    }
    else{
        copter.wp_nav->turn_into_wind_heading = (float)copter.initial_armed_bearing;
        SRV_Channels::set_output_pwm(SRV_Channel::k_egg_drop, fan_pwm_off);
        flag_fan_on = false;
        k = 0;
        _roll_sum = 0.0f;
        _pitch_sum = 0.0f;
        var_temp_dir = 0.0f;
        var_temp_gamma = 0.0f;
    }

    // Write wind direction packet into the SD card
    struct log_WIND pkt_wind_est = {
        LOG_PACKET_HEADER_INIT(LOG_WIND_MSG),
        time_stamp             : AP_HAL::micros64(),// - _last_read_ms),
        _wind_dir              : _wind_dir/100.0f,
        _wind_speed            : _wind_speed,
        _wind_dir_var          : var_wind_dir,
        _gamma_var             : var_gamma,
        _roll_sum              : avgR,
        _pitch_sum             : avgP,
        _yaw                   : _yaw
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
