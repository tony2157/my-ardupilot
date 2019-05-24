#include "Copter.h"
#include <utility>
#include <SRV_Channel/SRV_Channel.h>

//Humidity sensor Params
const int N_RH = 4;     //supports up to 4
float raw_H[4], rawRHt[4];
uint8_t healthy_RH[4];

//IMET sensor Params
const int N_imet = 4;   //supports up to 4
float resist[4], curr[4];
uint8_t healthy_imet[4];

//Fan control params
//It will run the fan at SERVO_MAX, which can be set in the params list
uint16_t fan_pwm_on = 100; // 40
uint16_t fan_pwm_off = 0;
bool _fan_status;

//Wind estimator Params
float wsA = 32.8;               //Coefficient A of the linear wind speed equation, from calibration
float wsB = -4.5;               //Coefficient B of the linear wind speed equation, from calibration
const int N = 60;               //filter order
float _wind_speed, _wind_dir;   //Estimated parameters
float _roll, _pitch, _yaw;      //UAS attitude
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
    memset(resist,0,sizeof(resist));
    memset(curr,0,sizeof(curr));
    memset(healthy_imet,0,sizeof(healthy_imet));
    memset(healthy_RH,0,sizeof(healthy_RH));

    //Initialize Wind estimator
    _wind_dir = 0.0f; _roll = 0.0f; _pitch = 0.0f; _yaw = 0.0f;
    _roll_sum = 0.0f; _pitch_sum = 0.0f;
    avgR = 0.0; avgP = 0.0;
    var_temp_dir = 0.0f; var_temp_gamma = 0.0f;
    _wind_dir = 0.0f;  _wind_speed = 0.0f;
    k = 0;
    memset(mean_aux_dir,0.0f,sizeof(mean_aux_dir));
    memset(var_aux_dir,1000.0f,sizeof(var_aux_dir));
    memset(mean_aux_speed,0.0f,sizeof(mean_aux_speed));
    memset(var_aux_speed,1000.0f,sizeof(var_aux_speed));

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
    // Read temperature
    curr[0] = copter.CASS_Imet[0].temperature();
    curr[1] = copter.CASS_Imet[1].temperature();
    curr[2] = copter.CASS_Imet[2].temperature();
    curr[3] = copter.CASS_Imet[3].temperature();
    // Read resistance
    resist[0] = copter.CASS_Imet[0].resistance();
    resist[1] = copter.CASS_Imet[1].resistance();
    resist[2] = copter.CASS_Imet[2].resistance();
    resist[3] = copter.CASS_Imet[3].resistance();
    // Read Health
    healthy_imet[0] = copter.CASS_Imet[0].healthy();
    healthy_imet[1] = copter.CASS_Imet[1].healthy();
    healthy_imet[2] = copter.CASS_Imet[2].healthy();
    healthy_imet[3] = copter.CASS_Imet[3].healthy();

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
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
        curr[0] = simT + sin(0.0005*m) * 0.001;
        curr[1] = 2 + curr[0] + sin(0.0006*m) * 0.001;
        curr[2] = 2 + curr[1] + sin(0.0004*m) * 0.001;
        curr[3] = 2 + curr[2] + sin(0.0007*m) * 0.001; 
    #endif

    // Write sensors packet into the SD card
    struct log_IMET pkt_temp = {
        LOG_PACKET_HEADER_INIT(LOG_IMET_MSG),
        time_stamp             : AP_HAL::micros64(),
        fan_status             : _fan_status,
        healthy1               : healthy_imet[0],
        healthy2               : healthy_imet[1],
        healthy3               : healthy_imet[2],
        healthy4               : healthy_imet[3],
        temperature1           : curr[0],
        temperature2           : curr[1],
        temperature3           : curr[2],
        temperature4           : curr[3],
        resist1                : resist[0],
        resist2                : resist[1],
        resist3                : resist[2],
        resist4                : resist[3]
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
    // Read Health
    healthy_RH[0] = copter.CASS_HYT271[0].healthy();
    healthy_RH[1] = copter.CASS_HYT271[1].healthy();
    healthy_RH[2] = copter.CASS_HYT271[2].healthy();
    healthy_RH[3] = copter.CASS_HYT271[3].healthy();

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        uint32_t m = AP_HAL::millis();
        rawRHt[0] = 298.15 + sin(0.0003*m) * 2;
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
        healthy1               : healthy_RH[0],
        healthy2               : healthy_RH[1],
        healthy3               : healthy_RH[2],
        healthy4               : healthy_RH[3],
        humidity1              : raw_H[0],
        humidity2              : raw_H[1],
        humidity3              : raw_H[2],
        humidity4              : raw_H[3],
        RHtemp1                : rawRHt[0],
        RHtemp2                : rawRHt[1],
        RHtemp3                : rawRHt[2],
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
    float body_wind_dir = 0.0f;
    float speed = 0.0f, dist_to_wp = 0.0f;
    Vector3f e_angles, vel_xyz;
    float alt;

    //copter.EKF2.getHAGL(alt);
    //alt = copter.inertial_nav.get_altitude();
    //copter.ahrs.get_hagl(alt);
    copter.ahrs.get_relative_position_D_home(alt);
    alt = -100.0f*alt;           // get AGL altitude in cm
    //printf("Alt: %5.2f \n",alt);

    //Start estimation after Copter takes off
    if(!ap.land_complete && copter.position_ok()){ // !arming.is_armed(), !ap.land_complete, motors->armed()

        //Fan Control    
        if(alt > 185.0f){
            SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_on);
            _fan_status = true;
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                printf("FAN ON \n");  
            #endif
        }
        else{
            if(alt < 140.0f){
                SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
                _fan_status = false;
                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    printf("FAN OFF \n");  
                #endif
            }
        }
        //uint16_t pwm;
        //SRV_Channels::get_output_pwm(SRV_Channel::k_egg_drop, pwm);
        //printf("PWM: %5.2f \n",(float)pwm);

        //Wind Estimator Algorithm
        if(alt > 200.0f){
            float aux; //Total area exposed to wind and aux variable

            //Current Attitude of the UAS
            copter.EKF2.getEulerAngles(-1,e_angles);
            _roll = e_angles.x;
            _pitch = e_angles.y;
            _yaw = e_angles.z;

            //Estimated horizontal velocity calculated by the EKF2
            //copter.EKF2.getVelNED(-1,vel_xyz);
            vel_xyz = copter.inertial_nav.get_velocity();
            speed = norm(vel_xyz.x,vel_xyz.y); // cm/s
            dist_to_wp = copter.wp_nav->get_wp_distance_to_destination(); // cm (horizontally)

            if(speed < 120.0f && dist_to_wp < 500){
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
                    avgP = _pitch_sum/N;
                    avgR = _roll_sum/N;
                    body_wind_dir = atan2f(sinf(avgR), -sinf(avgP));
                    var_gamma = var_temp_gamma/N - avgP*avgP;
                    var_wind_dir = var_temp_dir/N - body_wind_dir*body_wind_dir;
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
                        if(fabs(avgP)>0.03){
                            _wind_speed = wsA * sqrtf(tanf(acosf(cosf(avgP)*cosf(avgR)))) + wsB;
                        }
                        else{
                            _wind_speed = 0;
                        }
                        copter.cass_wind_speed = _wind_speed;
                    }

                    //Filter wind direction measurements
                    if(var_wind_dir < 1.5f){
                        _wind_dir = wrap_360_cd((_yaw + body_wind_dir)*5729.6f);
                        // Send wind direction to the Flight control 
                        if(alt>4.0f && var_wind_dir<0.8f && fabs(avgP)+fabs(avgR)>0.05f){
                            copter.cass_wind_direction = _wind_dir;
                        }
                        if(fabs(avgP)+fabs(avgR) < 0.05f) 
                        {
                            // If wind speeds are very slow or none at all (TODO: use wind speed estimation)
                            copter.cass_wind_direction = copter.wp_nav->get_yaw();
                        }
                    }
                    // Singularities avoidance algorithm
                    else if (var_wind_dir >= 1.5f && alt > 4.0f){
                        if(fabs(avgR) > fabs(avgP)){
                            if(avgR > 0){
                                copter.cass_wind_direction = wrap_360_cd(copter.cass_wind_direction + 1000.0f);
                            }
                            else{
                                copter.cass_wind_direction = wrap_360_cd(copter.cass_wind_direction - 1000.0f);
                            }
                        }
                        else{
                            if(avgP > 0){
                                copter.cass_wind_direction = wrap_360_cd(copter.cass_wind_direction + 18000.0f);
                            }
                            else{
                                copter.cass_wind_direction = copter.cass_wind_direction;
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
        //copter.wp_nav->turn_into_wind_heading = (float)copter.initial_armed_bearing;
        copter.cass_wind_direction = (float)copter.initial_armed_bearing;
        copter.cass_wind_speed = 0.0;
        SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
        _fan_status = false;
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
