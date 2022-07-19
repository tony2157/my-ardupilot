#include "Copter.h"
#include <utility>
#include <SRV_Channel/SRV_Channel.h>
#include <Filter/LPFrd.h>

//Fan control params
//It will run the fan at SERVO_MAX, which can be set in the params list
uint16_t fan_pwm_on = 100;
uint16_t fan_pwm_off = 0;
bool _fan_status;

//Wind estimator Params
float _wind_speed, _wind_dir;
float R13, R23, R33;
float last_yrate;
bool high_wind_flag;
uint32_t wvane_now;
//Declare digital LPF
LPFrdFloat filt_thrvec_x;
LPFrdFloat filt_thrvec_y;
LPFrdFloat filt_windspd;
//g.wind_vane_wsA -> Coefficient A of the linear wind speed equation, from calibration
//g.wind_vane_wsB -> Coefficient B of the linear wind speed equation, from calibration
//g.wind_vane_min_roll -> Minimum roll angle that the wind vane will correct (too low and the copter will oscilate)
//g.wind_vane_fine_rate -> Maximum yaw angle rate at which the Copter will rotate
//g.wind_vane_fine_gain -> Wind vane gain: higher values will increase the resposivness
//g.wind_vane_fs -> Wind vane sampling frequency, range 1 to 10Hz.

//Smart Vertical Profiling Battery monitor parameters
float Whc; // Energy consumed
float Whn; // Energy needed to fly home safely
float int_wvspd; // Wind speed history or memory while ascending
uint32_t vpbatt_now;
bool batt_home_ok;
bool batt_warning_flag;

//LB5900 global params
uint32_t LB_now;

//ARRC Gimbal function global params
uint32_t gimbal_now;
bool gimbal_execute;
uint8_t gimbal_iter;
const uint8_t gimbal_angle_span = 100;        // Must be an even number
const uint8_t gimbal_step = 10;              // Angle steps
const uint16_t gimbal_wait = 2200;           // Waiting time while gimbal is rotating
const uint16_t gimbal_sample_time = 2000;    // Sampling time at each angle step in milliseconds
float gimbal_probe_samples[gimbal_angle_span/gimbal_step + 1];
uint8_t gimbal_num_samples;
bool alignment_done;
Matrix3d rotm_step;

//AutoVP mission generation
uint32_t mission_now;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //Initialize Wind estimator
    _wind_dir = 0.0f;  _wind_speed = 0.0f;
    R13 = 0.0f; R23 = 0.0f;
    last_yrate = 0;
    high_wind_flag = false;
    wvane_now = AP_HAL::millis();

    //AutoVP initialize
    mission_now = AP_HAL::millis();

    //LB5900 initialize
    LB_now = AP_HAL::millis();

    //ARRC Gimbal params init
    gimbal_now = AP_HAL::millis();
    gimbal_execute = false;
    gimbal_iter = 0;
    gimbal_num_samples = 0;
    alignment_done = false;
    memset(gimbal_probe_samples, 0, (gimbal_angle_span/gimbal_step + 1) * sizeof(float));

    //Wind filter initialization
    float Fss;
    if(is_zero(fmodf(10,g2.user_parameters.get_wvane_fs()))){
        Fss = g2.user_parameters.get_wvane_fs();
    }
    else{
        Fss = 10.0f;
    }
    if(g2.user_parameters.get_wvane_cutoff() < 0.05){
        //Min Fc = 0.05 for stable yaw
        filt_thrvec_x.set_cutoff_frequency(Fss,0.05);
        filt_thrvec_y.set_cutoff_frequency(Fss,0.05);
        filt_windspd.set_cutoff_frequency(Fss,0.5);
    }
    else{
        //Initialize Butterworth filter
        filt_thrvec_x.set_cutoff_frequency(Fss,g2.user_parameters.get_wvane_cutoff());
        filt_thrvec_y.set_cutoff_frequency(Fss,g2.user_parameters.get_wvane_cutoff());
        filt_windspd.set_cutoff_frequency(Fss,0.5);
    }

    //VPBatt_monitor initilize
    Whc = Whn = int_wvspd = 0;
    batt_home_ok = true;
    batt_warning_flag = false;
    vpbatt_now = AP_HAL::millis();

    // Initialize Fan Control
    SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
    _fan_status = false;
}
#endif

#ifdef USER_ARRCLB5900_LOOP
void Copter::user_LB5900_logger()
{
    // Read Power in dBm. Write sensors packet into the SD card
    // LB5900 Power Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
    struct log_LB5900 pkt_temp = {
        LOG_PACKET_HEADER_INIT(LOG_LB5900_MSG),
        time_stamp              : AP_HAL::micros64(),                   //Store time in microseconds
        healthy                 : copter.ARRC_LB5900.healthy(),         //Store sensor health
        power                   : copter.ARRC_LB5900.power_measure(),   //Store power in dBm
    };
    logger.WriteBlock(&pkt_temp, sizeof(pkt_temp));   //Send package to SD card

    // Print desired params for Debugging
    // if (AP_HAL::millis() - LB_now > 2000){

    //     const char* (mrate[1])[4] = 
    //     {
    //         "NORMAL",   // 20 readings per sec
    //         "DOUBLE",   // 40 readings per sec
    //         "FAST",     // 110 readings per sec (disallows average count)
    //         "SUPER"     // 110 readings per sec (allows average count)
    //     };

    //     gcs().send_text(MAV_SEVERITY_INFO,"LB health: %d",(uint8_t)copter.ARRC_LB5900.healthy());
    //     gcs().send_text(MAV_SEVERITY_INFO,"LB power: %d",(uint8_t)copter.ARRC_LB5900.power_measure());

    //     uint16_t freq = g2.user_parameters.get_lb5900_freq();
    //     uint16_t avg_cnt = g2.user_parameters.get_lb5900_avg_cnt();
    //     uint8_t rate = g2.user_parameters.get_lb5900_mrate();

    //     char FREQ[10 + sizeof(char)] = "FREQ ";
    //     char AVG_CNT[17 + sizeof(char)] = "SENS:AVER:COUN ";
    //     char MRATE[16 + sizeof(char)] = "SENS:MRAT ";
    //     char temp[5 + sizeof(char)];

    //     snprintf(temp,6,"%d",freq);
    //     strcat(FREQ, temp);
    //     strcat(FREQ, " MHZ");
    //     snprintf(temp,6,"%d",avg_cnt);
    //     strcat(AVG_CNT, temp);
    //     strcat(MRATE, mrate[0][rate]);

    //     gcs().send_text(MAV_SEVERITY_INFO,"%s",FREQ);
    //     gcs().send_text(MAV_SEVERITY_INFO,"%s",AVG_CNT);
    //     gcs().send_text(MAV_SEVERITY_INFO,"%s",MRATE);

    //     LB_now = AP_HAL::millis();
    // }
}

#endif

#ifdef USER_ARRCRFE_LOOP
void Copter::user_RFE_logger()
{
    // Read Power in dBm. Write sensors packet into the SD card
    // RFExplorer Power Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
    struct log_RFE pkt_temp = {
        LOG_PACKET_HEADER_INIT(LOG_RFE_MSG),
        time_stamp              : copter.ARRC_RFE.get_timestamp(),      //Store time in microseconds
        freq                    : copter.ARRC_RFE.get_freq(),           //Store sampled frequency in Hz
        power                   : copter.ARRC_RFE.get_power(),          //Store sampled power in dBm
    };
    logger.WriteBlock(&pkt_temp, sizeof(pkt_temp));   //Send package to SD card
}

#endif

#ifdef USER_GIMBAL_LOOP
void Copter::user_ARRC_gimbal()
{
    //float elev = copter.camera_mount.get_AUT_elevation();
    Vector3d axis = {-1.0, 0.0, 0.0}; // Rotate about the axis that is pointing to the AUT (Gimbal frame)
    uint8_t N = gimbal_angle_span/2;

    if(gimbal_execute == true){

        // Set Gimbal at initial rotation
        rotm_step.from_axis_angle(axis, ((double)(-N))*DEG_TO_RAD);
        copter.camera_mount.set_RotM_offset(rotm_step);

        if((AP_HAL::millis() - gimbal_now) < 4000){ return;}

        repeat:
        if(gimbal_iter <= 2*N){

            // rotate gimbal at predefined angle steps
            rotm_step.from_axis_angle(axis, ((double)(-N+gimbal_iter))*DEG_TO_RAD);
            copter.camera_mount.set_RotM_offset(rotm_step);

            if((AP_HAL::millis() - gimbal_now) < (uint32_t)(4000 + gimbal_wait*(gimbal_iter/gimbal_step+1))){ 
                return;
            }
            if((AP_HAL::millis() - gimbal_now) < (uint32_t)(4000 + (gimbal_sample_time+gimbal_wait)*(gimbal_iter/gimbal_step+1))){ 
                gimbal_probe_samples[gimbal_iter/gimbal_step] = gimbal_probe_samples[gimbal_iter/gimbal_step] + copter.ARRC_RFE.get_power();
                gimbal_num_samples++;
                return;
            }
            gimbal_probe_samples[gimbal_iter/gimbal_step] /= gimbal_num_samples;
            gimbal_num_samples = 0;
            gimbal_iter = gimbal_iter + gimbal_step;
            goto repeat;
        }

        // Fake input for debugging
        // float y[31] = {16.85, 17.2, 17.53, 17.84, 18.13, 18.4, 18.65, 
        //                 18.88, 19.09, 19.28, 19.45, 19.6, 19.73, 19.84, 
        //                 19.93, 20, 20.05, 20.08, 20.09, 20.08, 20.05, 20,
        //                 19.93, 19.84, 19.73, 19.6, 19.45, 19.28, 19.09,
        //                 18.88, 18.65};

        int8_t i,j,k;
        int8_t n = gimbal_angle_span/gimbal_step;

        float x[n + 1];
        for(i = 0; i<=2*N; i=i+gimbal_step){
            x[i/gimbal_step] = i - N;
        }

        // Polynomial Fit using Least Squares

        float X[5];                             //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
        for (i=0;i<5;i++)
        {
            X[i]=0;
            for (j=0;j<n+1;j++)
                X[i]=X[i]+pow(x[j],i);          //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
        }
        float B[3][4];                     //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
        for (i=0;i<=2;i++)
            for (j=0;j<=2;j++)
                B[i][j]=X[i+j];

        float Y[3];                             //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
        for (i=0;i<3;i++)
        {    
            Y[i]=0;
            for (j=0;j<n+1;j++)
            Y[i]=Y[i]+pow(x[j],i)*gimbal_probe_samples[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
        }

        for (i=0;i<=2;i++)
            B[i][3]=Y[i];                       //load the values of Y as the last column of B(Normal Matrix but augmented) 
        
        for (i=0;i<3;i++)                       //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
            for (k=i+1;k<3;k++)
                if (B[i][i]<B[k][i])
                    for (j=0;j<=3;j++)
                    {
                        float temp=B[i][j];
                        B[i][j]=B[k][j];
                        B[k][j]=temp;
                    }

        for (i=0;i<2;i++)                       //loop to perform the gauss elimination
            for (k=i+1;k<3;k++)
                {
                    float t=B[k][i]/B[i][i];
                    for (j=0;j<=3;j++)
                        B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
                }

        float a[3] = {0,0,0};
        for (i=2;i>=0;i--)                      //back-substitution
        {                                       //x is an array whose values correspond to the values of x,y,z..
            a[i]=B[i][3];                       //make the variable to be calculated equal to the rhs of the last equation
            for (j=0;j<3;j++)
                if (j!=i)                       //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                    a[i]=a[i]-B[i][j]*a[j];
            a[i]=a[i]/B[i][i];                  //now finally divide the rhs by the coefficient of the variable to be calculated
        }

        // Check if we got a local maxima. Otherwise, the alignment failed
        if(a[2] > 0 || is_zero(a[2])){
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment failed: No Maxima");
            gimbal_execute = false;
            return;
        }

        // Evaluate resulting curve fitting at each measured angle
        float y[n + 1];
        for(i = 0; i<=n; i++){
            y[i] = a[2]*x[i]*x[i] + a[1]*x[i] + a[0];
        }

        // Compute correlation coefficient
        float sum_X = 0, sum_Y = 0, sum_XY = 0;
        float squareSum_X = 0, squareSum_Y = 0;

        for (i = 0; i <= n; i++)
        {
            // sum of elements of array X (sampled points)
            sum_X = sum_X + gimbal_probe_samples[i];
    
            // sum of elements of array Y (output of the computed curve)
            sum_Y = sum_Y + y[i];
    
            // sum of X[i] * Y[i].
            sum_XY = sum_XY + gimbal_probe_samples[i] * y[i];
    
            // sum of square of array elements.
            squareSum_X = squareSum_X + gimbal_probe_samples[i] * gimbal_probe_samples[i];
            squareSum_Y = squareSum_Y + y[i] * y[i];
        }
  
        // use formula for calculating correlation coefficient.
        float corr = (float)(n * sum_XY - sum_X * sum_Y) 
                    / sqrtf((n * squareSum_X - sum_X * sum_X) 
                        * (n * squareSum_Y - sum_Y * sum_Y));

        // Check the correlation coefficient. Alignment failed if corr is too low
        if(fabsf(corr) < 0.85){
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment failed: R = %f",corr);
            gimbal_execute = false;
            return;
        }

        // Resultant angle offset
        float angle_offset = -a[1]/(2*a[2]);

        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment correction: %f deg",angle_offset);
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment success: R = %f",corr);

        // Send and apply resultant rotation matrix offset to the gimbal
        rotm_step.from_axis_angle(axis, ((double)angle_offset)*DEG_TO_RAD);
        copter.camera_mount.set_RotM_offset(rotm_step);

        gimbal_execute = false;
    }
}
#endif

#ifdef USER_GIMBAL_SIM_LOOP
void Copter::user_ARRC_gimbal_sim()
{
    Location this_loc;
    Location target;
    if (!AP::ahrs().get_position(this_loc)) {
        return;
    }
    
    if(copter.position_ok()){
        target = AP::ahrs().get_home();
    }
    else {
        return;
    }

    // Haversine formula
    float curr_lat = this_loc.lat*1.0e-7f*M_PI/180.0f;
    float tar_lat = target.lat*1.0e-7f*M_PI/180.0f;
    float delta_lat = tar_lat - curr_lat;
    float delta_lng = Location::diff_longitude(target.lng,this_loc.lng)*1.0e-7f*M_PI/180.0f;
    float a = sinf(delta_lat/2)*sinf(delta_lat/2) + cosf(curr_lat)*cosf(tar_lat)*sinf(delta_lng/2)*sinf(delta_lng/2);

    // Compute distance to target
    float target_distance = 2.0f*RADIUS_OF_EARTH*atan2f(sqrtf(a),sqrtf(1-a)); // in meters

    // Compute bearing to target
    float y = sinf(delta_lng)*cosf(tar_lat);
    float x = cosf(curr_lat)*sinf(tar_lat) - sinf(curr_lat)*cosf(tar_lat)*cosf(delta_lng);
    float bearing = atan2f(y, x);

    float fixed_yaw = g2.user_parameters.get_user_sensor1()*DEG_TO_RAD;
    float ang_diff = wrap_PI(bearing - fixed_yaw);

    // Compute target vector x-y components in the target's reference frame (NWU)
    y = target_distance*(sinf(bearing)*cosf(fixed_yaw) - cosf(bearing)*sinf(fixed_yaw)); // Aligned with West when fixed_yaw = 0
    x = -target_distance*(sinf(bearing)*sinf(fixed_yaw) + cosf(bearing)*cosf(fixed_yaw)); // Aligned with North when fixed_yaw = 0

    int32_t target_alt_cm = 0;
    if (!target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm)) {
        return;
    }
    int32_t current_alt_cm = 0;
    if (!this_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt_cm)) {
        return;
    }

    // Compute height difference
    float z = (float)(current_alt_cm - target_alt_cm)/100.0f; // in meters

    //Compute distance and slope wrt target
    float horzdist2target = current_loc.get_distance(target);
    float dist2target = sqrtf(horzdist2target*horzdist2target + (float)(z*z));
    float slope = 0;
    if(!is_zero(horzdist2target)) slope = fabsf((float)z/horzdist2target);

    // initialise all angles to zero
    Vector3f angles_to_target_rad;
    angles_to_target_rad.zero();

    // Matrix3f rotM_offset(0.9848, -0.1736, 0.0000,
    //                      0.1736, 0.9848, 0.0000,
    //                      0.0000, 0.0000, 1.0000);

    Matrix3f rotM_offset;                
    rotM_offset.identity();

    if(dist2target > 10){
        if(g2.user_parameters.get_user_sensor2() == 0 || slope < 0.38f){

            // Original ArduPilot mode

            // tilt calcs
            angles_to_target_rad.y = atan2f(-z, target_distance);

            // roll is leveled to the ground

            // pan calcs
            angles_to_target_rad.z = bearing;
            angles_to_target_rad.z = wrap_PI((angles_to_target_rad.z - (float)AP::ahrs().yaw));
        }
        else if(g2.user_parameters.get_user_sensor2() == 1){

            // Point gimbal straight down without corrections

            // tilt calcs. Fixed 
            angles_to_target_rad.y = M_PI_2;
            
            // roll calcs. Fixed
            angles_to_target_rad.x = 0;

            // pan calcs. Fixed and equal to user param
            angles_to_target_rad.z = fixed_yaw;
            angles_to_target_rad.z = wrap_PI(angles_to_target_rad.z - AP::ahrs().yaw);
        }
        else if(g2.user_parameters.get_user_sensor2() == 2){

            // Old Technique (less accurate)

            // tilt calcs
            angles_to_target_rad.y = atan2f(z, target_distance*cosf(ang_diff));
            
            // roll calcs
            angles_to_target_rad.x = atan2f(z, target_distance*sinf(ang_diff)) + M_PI_2;

            // pan is set to a fixed value defined by user
            angles_to_target_rad.z = fixed_yaw;
            angles_to_target_rad.z =  wrap_PI(angles_to_target_rad.z - AP::ahrs().yaw);
        }
        else if(g2.user_parameters.get_user_sensor2() == 3){

            // Hpol aligned mode

            float D = sqrtf(x*x + y*y + z*z);
            float A = sqrtf(x*x+z*z);

            Matrix3f RotM( -x/D,         -y/D,      -z/D,
                           -x*y/(A*D),   A/D,       -y*z/(A*D),
                           z/A,          0,         -x/A   );

            RotM = RotM*rotM_offset;

            // tilt calcs = atan2(Reb(1,3),Reb(3,3))
            angles_to_target_rad.y = atan2f(RotM.a.z, RotM.c.z);
            
            // roll calcs = atan2(-Reb(2,3),sqrt(1-Reb(2,3)^2))
            angles_to_target_rad.x = atan2f(-RotM.b.z, sqrtf(1.0 - RotM.b.z*RotM.b.z));

            // pan calcs = atan2(Reb(2,1),Reb(2,2))
            angles_to_target_rad.z = atan2f(RotM.b.x,RotM.b.y) + fixed_yaw;
            angles_to_target_rad.z = wrap_PI((angles_to_target_rad.z - (float)AP::ahrs().yaw));
        }
        else if(g2.user_parameters.get_user_sensor2() == 4){

            // Vpol aligned mode

            float el = g2.user_parameters.get_user_sensor3()*DEG_TO_RAD;

            float D = sqrtf(x*x + y*y + z*z);
            float A = sqrt((x*x - z*z)*cos(el)*cos(el) + y*y + z*z - x*z*sin(2*el));
            float B = sqrtf((x*x- z*z)*cosf(2*el) + x*x + 2*y*y + z*z - 2*x*z*sinf(2*el));

            Matrix3f RotM( -x/D,                                                        -y/D,                                                   -z/D,
                           y*cosf(el)/A,                                                -(x*cosf(el)-z*sinf(el))/A,                            -y*sinf(el)/A,
                           (M_SQRT2*((y*y*+z*z)*sinf(el)-x*z*cosf(el)))/(D*B),         -(M_SQRT2*y*(z*cosf(el)+x*sinf(el)))/(D*B),            M_SQRT2*((x*x+y*y)*cosf(el)-x*z*sinf(el))*B/(2*D*A*A)  );

            RotM = RotM*rotM_offset;

            // tilt calcs = atan2(Reb(1,3),Reb(3,3))
            angles_to_target_rad.y = atan2f(RotM.a.z, RotM.c.z);
            
            // roll calcs = atan2(-Reb(2,3),sqrt(1-Reb(2,3)^2))
            angles_to_target_rad.x = atan2f(-RotM.b.z, sqrtf(1.0 - RotM.b.z*RotM.b.z));

            // pan calcs = atan2(Reb(2,1),Reb(2,2))
            angles_to_target_rad.z = atan2f(RotM.b.x,RotM.b.y) + fixed_yaw;
            angles_to_target_rad.z = wrap_PI((angles_to_target_rad.z - (float)AP::ahrs().yaw));
        }
    }
    
    printf("targer_dist: %5.2f \n",target_distance);
    printf("Target_Z: %5.2f \n",z);
    printf("Target_X: %5.2f \n",x);
    printf("Target_Y: %5.2f \n",y);
    printf("Ang_diff: %5.2f \n",ang_diff);
    printf("Gimbal_Roll: %5.2f \n",((float)angles_to_target_rad.x)*RAD_TO_DEG);
    printf("Gimbal_Pitch: %5.2f \n",((float)angles_to_target_rad.y)*RAD_TO_DEG);
    printf("Gimbal_Yaw: %5.2f \n",((float)angles_to_target_rad.z)*RAD_TO_DEG);
}
#endif

#ifdef USER_VPBATT_MNTR_LOOP
void Copter::user_vpbatt_monitor()
{
    // Smart Battery Management - Use it only for Vertical profiling
    float alt;
    float dt = (float)(AP_HAL::millis() - vpbatt_now);
    // Compute loop only when flying
    if(!ap.land_complete){
        // Enter loop every 100 milliseconds
        if(dt >= 100.0f){
            // Calculate energy consumed in Watt-hour
            float current;
            if(battery.current_amps(current)){
                Whc = Whc + battery.voltage()*current*dt/3.6e6f;
            }

            //Compute the temporal integration of the wind speed (works as a memory)
            Vector3f velocity;
            copter.ahrs.get_velocity_NED(velocity);
            if(velocity[2] < 0){
                int_wvspd = int_wvspd + _wind_speed*dt/1000.0f;
            }

            // Calculate the Descent-Energy-consumption per meter height (function of wind speed)
            float Whm = 2.0e-6f*int_wvspd + 8.0e-3f;
            // Constrain lower values
            Whm = Whm > 0.0105f ? Whm : 0.0105f;
            
            // Get current altitude in meters
            copter.ahrs.get_relative_position_D_home(alt);
            alt = -1.0f*alt;
            // Calculate energy needed to get home safely
            Whn = Whm*alt;
            
            // Estimate the total energy used (percentage)
            // vpbatt_reserve is the desired batt percentage after landing
            float Wh_tot = (Whc + Whn)/g2.user_parameters.get_vpbatt_wh() + g2.user_parameters.get_vpbatt_reserve()/100.0f + 0.05f;

            //Switch to RTL automatically if battery reaches critical remaining energy
            if(!is_zero(g2.user_parameters.get_vpbatt_wh())){
                // Issue a warning once when the battery altitude range is over 85%
                if(Wh_tot >= 0.85f && batt_warning_flag == false){
                    // It will still warn, even if the function is disabled
                    if(!is_zero(g2.user_parameters.get_vpbatt_enabled())){
                        gcs().send_text(MAV_SEVERITY_WARNING, "Over 85 Batt range");
                    }
                    batt_warning_flag = true;
                }

                // Trigger RTL when max battery altitude range is reached
                if(Wh_tot >= 1.0f && batt_home_ok == true){
                    gcs().send_text(MAV_SEVERITY_WARNING, "Max Batt range: Switch to RTL");
                    // It will still warn, even if the function is disabled
                    if(!is_zero(g2.user_parameters.get_vpbatt_enabled())){
                        copter.set_mode(Mode::Number::RTL, ModeReason::UNKNOWN);
                    }
                    batt_home_ok = false;
                }
            }
            // Update time
            vpbatt_now = AP_HAL::millis();

            //Print on terminal for debugging
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            // printf("Whc: %5.2f \n",Whc);
            // printf("Vel_Z: %5.2f \n",velocity[2]);
            // printf("int_wvspd: %5.4f \n",int_wvspd);
            // printf("Whm: %5.4f \n",Whm);
            // printf("Whn: %5.2f \n",Whn);
            // printf("Wh_tot: %5.2f \n",Wh_tot);
            #endif
        }
    }
    else{
        vpbatt_now = AP_HAL::millis();
        int_wvspd = 0.0f;
    }
}
#endif

#ifdef USER_THERMOHYGROMETER_LOOP
void Copter::user_thermohygrometer_logger()
{
    // Read Temperature, Resistance and Health. Write sensors packet into the SD card
    // Temperature Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
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
    logger.WriteBlock(&pkt_temp, sizeof(pkt_temp));   //Send package to SD card

    // Read Rel. Humidity, Temperature and Health. Write sensors packet into the SD card
    // Relative Humidity Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
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
    logger.WriteBlock(&pkt_RH, sizeof(pkt_RH));   //Send package to SD card
}
#endif

#ifdef USER_WIND_LOOP
void Copter::user_wind_vane()
{
    //Run algo after Copter takes off
    if(!ap.land_complete && copter.position_ok()){

        //Fan Control ////////////////////////////////////////////////////////////////////////////////////////

        //Get AGL altitude in cm
        float alt;
        copter.ahrs.get_relative_position_D_home(alt);
        alt = -100.0f*alt;
   
        //Smart fan on/off logic
        if(alt > 185.0f && SRV_Channels::get_output_scaled(SRV_Channel::k_egg_drop) < 50){
            SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_on);
            _fan_status = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Scoop Fan activated");
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

        //Wind Estimator Algorithm //////////////////////////////////////////////////////////////////////////

        //Get thurst vector elements from the rotation matrix
        R13 = -1*copter.ahrs.get_rotation_body_to_ned().a.z;
        R23 = -1*copter.ahrs.get_rotation_body_to_ned().b.z;
        R33 = -1*copter.ahrs.get_rotation_body_to_ned().c.z;

        //Wind vane loop starts here. Loop frequency is defined by WVANE_FS param in Hz
        if((AP_HAL::millis() - wvane_now) >= (uint32_t)(1000/g2.user_parameters.get_wvane_fs())){
            //Apply Butterworth LPF on each element
            float thrvec_x, thrvec_y;
            thrvec_x = filt_thrvec_x.apply(R13);
            thrvec_y = filt_thrvec_y.apply(R23);

            //Determine wind direction by trigonometry (thrust vector tilt)
            float wind_psi = fmodf(atan2f(thrvec_y,thrvec_x),2*M_PI)*RAD_TO_DEG + g2.user_parameters.get_wvane_offset();
            _wind_dir = wrap_360_cd(wind_psi*100.0f);

            //Estimate wind speed with filtered parameters
            float R_xy = safe_sqrt(R13*R13 + R23*R23);
            _wind_speed = g2.user_parameters.get_wvane_wsA() * fabsf(R_xy/R33) + g2.user_parameters.get_wvane_wsB()*safe_sqrt(fabsf(R_xy/R33));
            _wind_speed = _wind_speed < 0 ? 0.0f : _wind_speed;
            _wind_speed = filt_windspd.apply(_wind_speed);

            //Get current velocity
            Vector3f vel_xyz = copter.inertial_nav.get_velocity(); // NEU convention
            float tyaw = copter.wp_nav->get_yaw()*DEG_TO_RAD/100.0f;
            float speed_y = vel_xyz.y*cosf(tyaw) - vel_xyz.x*sinf(tyaw); // Get lateral velocity in body frame
            float speed = norm(vel_xyz.x,vel_xyz.y); 
            
            //Wind vane is active when flying horizontally steady and wind speed is perceivable
            //Condition when ascending
            if(fabsf(speed_y) < 150.0f && _wind_speed > 1.0f && vel_xyz[2] >= 0.0f){
                //Min altitude and speed at which the yaw command is sent
                if(alt>400.0f && speed<(fabsf(speed_y)+300.0f)){ 
                    //Send estimated wind direction to the autopilot
                    copter.cass_wind_direction = _wind_dir;
                    copter.cass_wind_speed = _wind_speed;
                }
                else{
                    //Send neutral values
                    copter.cass_wind_direction = copter.wp_nav->get_yaw();
                    copter.cass_wind_speed = 0.0f;
                }
            }
            //Condition when descending
            else if (fabsf(speed_y) < 150.0f && _wind_speed > 3.0f && vel_xyz[2] < 0.0f){
                if(alt>600.0f && speed<(fabsf(speed_y)+300.0f)){ 
                    //Send estimated wind direction to the autopilot
                    copter.cass_wind_direction = _wind_dir;
                    copter.cass_wind_speed = _wind_speed;
                }
                else{
                    //Do nothing - keep yaw equal to last wind direction estimate
                    copter.cass_wind_speed = 0.0f;
                }
            }
            else{
                //Reset 1st order filter
                last_yrate = 0.0f;
            }

            //Switch to RTL automatically if wind speed is too high (in m/s)
            //If tolerance is set to zero then auto RTL is disabled but it will still warn if enabled
            if(!is_zero(g2.user_parameters.get_wvane_spd_tol())){
                if(_wind_speed > g2.user_parameters.get_wvane_spd_tol() && high_wind_flag == false && copter.flightmode->is_autopilot()){
                    gcs().send_text(MAV_SEVERITY_WARNING, "Warning high wind: Switch to RTL");
                    if(!is_zero(g2.user_parameters.get_wvane_enabled())){
                        copter.set_mode(Mode::Number::RTL, ModeReason::UNKNOWN);
                    }
                    high_wind_flag = true;
                }
                else if(_wind_speed < (g2.user_parameters.get_wvane_spd_tol() - 3.0f) && high_wind_flag == true){
                    high_wind_flag = false;
                    gcs().send_text(MAV_SEVERITY_INFO, "High wind warning cleared");
                }
            }

            //Update last loop time
            wvane_now = AP_HAL::millis();
        }
        
    }
    else{
        //Reset all global parameters to default values
        copter.cass_wind_direction = (float)copter.initial_armed_bearing;
        copter.cass_wind_speed = 0.0f;
        SRV_Channels::set_output_scaled(SRV_Channel::k_egg_drop, fan_pwm_off);
        last_yrate = 0;
        _fan_status = false;
        filt_thrvec_x.reset();
        filt_thrvec_y.reset();
        filt_windspd.reset();
    }

    // Wind Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
    // Write wind direction packet into the SD card
    struct log_WIND pkt_wind_est = {
        LOG_PACKET_HEADER_INIT(LOG_WIND_MSG),
        time_stamp             : AP_HAL::micros64(),
        _wind_dir              : _wind_dir/100,
        _wind_speed            : _wind_speed,
        _R13                   : R13,
        _R23                   : R23,
        _R33                   : R33
    };
    logger.WriteBlock(&pkt_wind_est, sizeof(pkt_wind_est));
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1()
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    // Code runs when switch is toggled HIGH (pwm > 1800)
    AP_Mission::Mission_Command cmd;
    float max_alt = g2.user_parameters.get_autovp_max_alt()*100; //convert to cm

    // Check if drone is grounded and ready to create a mission
    if(ap.land_complete && copter.position_ok() && (AP_HAL::millis() - mission_now) > 5000){

        // Get current position
        int32_t vp_lat = copter.current_loc.lat; // ahrs.get_home().lat;
        int32_t vp_lng = copter.current_loc.lng; // ahrs.get_home().lng;

        // clear mission
        if(!copter.mode_auto.mission.clear()){
            gcs().send_text(MAV_SEVERITY_WARNING, "AutoVP: Mission could not be cleared");
        }
        
        // Command #0 : home
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.content.location = Location{
                                    vp_lat,
                                    vp_lng,
                                    0,
                                    Location::AltFrame::ABOVE_HOME};
        if (!copter.mode_auto.mission.add_cmd(cmd)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "AutoVP: failed to create mission");
        }

        // Command #1 : take-off to 5m
        cmd.id = MAV_CMD_NAV_TAKEOFF;
        cmd.p1 = 0;
        cmd.content.location = Location{
                                    0,
                                    0,
                                    300,
                                    Location::AltFrame::ABOVE_HOME};
        if (!copter.mode_auto.mission.add_cmd(cmd)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "AutoVP: failed to create mission");
        }

        // Command #2 : Bottom waypoint at 10m
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 0;
        cmd.content.location = Location{
                                    vp_lat,
                                    vp_lng,
                                    1000,
                                    Location::AltFrame::ABOVE_HOME};
        if (!copter.mode_auto.mission.add_cmd(cmd)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "AutoVP: failed to create mission");
        }

        // Constrain target altitude
        if(max_alt > 180000){
            max_alt = 180000;
            gcs().send_text(MAV_SEVERITY_INFO, "AutoVP: Max Alt set to 1800m");
        }
        if(max_alt < 1000){
            max_alt = 1000;
            gcs().send_text(MAV_SEVERITY_INFO, "AutoVP: Max Alt set to 10m");
        }

        // Command #3 : Top waypoint at desired altitude (from parameter)
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 0;
        cmd.content.location = Location{
                                    vp_lat,
                                    vp_lng,
                                    (int32_t)max_alt,
                                    Location::AltFrame::ABOVE_HOME};
        if (!copter.mode_auto.mission.add_cmd(cmd)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "AutoVP: failed to create mission");
        }

        // Command #4 : RTL
        cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
        cmd.p1 = 0;
        cmd.content.location = Location{
                                    0,
                                    0,
                                    0,
                                    Location::AltFrame::ABOVE_HOME};
        if (!copter.mode_auto.mission.add_cmd(cmd)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "AutoVP: failed to create mission");
        }

        // Send successful creation message
        gcs().send_text(MAV_SEVERITY_INFO, "AutoVP mission received");
        gcs().send_text(MAV_SEVERITY_INFO, "Target alt: %f m",max_alt/100);

        mission_now = AP_HAL::millis();
    }
    else{
        // Send unable to create mission message warning
        gcs().send_text(MAV_SEVERITY_WARNING, "AutoVP: Unable to create mission, EKF not ready");
    }
}

void Copter::userhook_auxSwitch2()
{
    // Execution of the ARRC gimbal movement
    // put your aux switch #2 handler here (CHx_OPT = 48)

    if(alignment_done == false ){
        gcs().send_text(MAV_SEVERITY_INFO, "Executing Gimbal alignment");
        memset(gimbal_probe_samples, 0, (gimbal_angle_span/gimbal_step + 1) * sizeof(float));
        gimbal_num_samples = 0;
        gimbal_iter = 0;
        gimbal_now = AP_HAL::millis();
        gimbal_execute = true;
        alignment_done = true;
    }
    else{
        rotm_step.identity();
        copter.camera_mount.set_RotM_offset(rotm_step);
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal RotM offset cleared");
        alignment_done = false;
        gimbal_execute = false;
    }
}

void Copter::userhook_auxSwitch3()
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif