#include "Copter.h"

//ARRC Gimbal function global params
uint32_t gimbal_now;
uint32_t switch_pause;
bool gimbal_execute;
uint8_t gimbal_iter;
const uint8_t gimbal_angle_span = 64;        // Must be an even number
const uint8_t gimbal_step = 8;              // Angle steps
const uint16_t gimbal_init_wait = 4000;      // Gimbal initial waiting time
const uint16_t gimbal_wait = 3000;           // Waiting time while gimbal is rotating
const uint16_t gimbal_sample_time = 10000;    // Sampling time at each angle step in milliseconds
float gimbal_probe_samples[gimbal_angle_span/gimbal_step + 1];
uint8_t gimbal_num_samples;
bool alignment_done;
Matrix3d rotm_step;

float mean(const float* v, int n);
float dot_product(const float* v1, const float* v2, int n);
float standard_deviation(const float* v, float mean, int n);
float correlation(const float* v1, const float* v2, int n);

uint8_t kek;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //ARRC Gimbal params init
    gimbal_now = AP_HAL::millis();
    switch_pause = AP_HAL::millis();
    gimbal_execute = false;
    gimbal_iter = 0;
    gimbal_num_samples = 0;
    alignment_done = true;
    memset(gimbal_probe_samples, 0, (gimbal_angle_span/gimbal_step + 1) * sizeof(float));

    kek = 0;
}
#endif

#ifdef USER_GIMBAL_LOOP
void Copter::user_ARRC_gimbal()
{
    if(gimbal_execute == true){

        // Rotate about the axis that is pointing to the AUT (Gimbal frame)
        Vector3d axis = {-1.0, 0.0, 0.0};
        uint8_t N = gimbal_angle_span/2;

        // Set Gimbal to initial rotation
        rotm_step.from_axis_angle(axis, ((double)(-N))*DEG_TO_RAD);
        copter.camera_mount.set_RotM_offset(rotm_step);

        // Wait for a certain amount of time before starting the measurements
        if((AP_HAL::millis() - gimbal_now) < gimbal_init_wait){ return;}

        repeat:
        if(gimbal_iter <= 2*N){

            // rotate gimbal to the next angle step
            rotm_step.from_axis_angle(axis, ((double)(-N+gimbal_iter))*DEG_TO_RAD);
            copter.camera_mount.set_RotM_offset(rotm_step);

            // Wait for the gimbal to complete rotating
            if((AP_HAL::millis() - gimbal_now) < (uint32_t)(gimbal_init_wait + gimbal_wait*(gimbal_iter/gimbal_step+1))){ 
                return;
            }

            // Collect power measurements during a time period
            if((AP_HAL::millis() - gimbal_now) < (uint32_t)(gimbal_init_wait + (gimbal_sample_time+gimbal_wait)*(gimbal_iter/gimbal_step+1))){ 
                gimbal_probe_samples[gimbal_iter/gimbal_step] = gimbal_probe_samples[gimbal_iter/gimbal_step] + copter.ARRC_SDR.get_pwr_c();
                gimbal_num_samples++;
                return;
            }

            // Average the collected power measurements
            gimbal_probe_samples[gimbal_iter/gimbal_step] /= gimbal_num_samples;
            gimbal_num_samples = 0;

            // Reapeat the process until completing all angle steps
            gimbal_iter = gimbal_iter + gimbal_step;
            goto repeat;
        }

        // Simulated data points for code test and debugging
        // if (kek == 0){
        //     float newValues[] = {-10.6525, -10.2897, -10.0700, -10.0001, -10.0821, -10.3136, -10.6874}; // -0.2 deg offset, measured: -0.200017
        //     for(int i = 0; i < 7; i++) {
        //         gimbal_probe_samples[i] = newValues[i];
        //     }
        //     kek = kek + 1;
        // }
        // else if (kek == 1){
        //     float newValues[] = {-10.5450, -10.2185, -10.0373, -10.0069, -10.1281, -10.3975, -10.8066}; // -1.5 deg offset, measured: -1.49966
        //     for(int i = 0; i < 7; i++) {
        //         gimbal_probe_samples[i] = newValues[i];
        //     }
        //     kek = kek + 1;
        // }
        // else{
        //     float newValues[] = {-10.0760, -10.0000, -10.0760, -10.3015, -10.6699, -11.1698, -11.7861}; // -10 deg offset, measured: -10.00034
        //     for(int i = 0; i < 7; i++) {
        //         gimbal_probe_samples[i] = newValues[i];
        //     }
        //     kek = 0;
        // }

        // Create array of angle steps corresponding to each power measurement
        // This method is called the Harmonic regression (or trigonometric regression)
        // The ceoficients are calculated using Ordinary Least Square (OLS) method
        // Polarization Loss factor is assumed equal to cos(phi)^2 which can be expressed as (1 + cos(2*phi))/2

        int8_t n = gimbal_angle_span/gimbal_step;
        float x[n+1];
        for(int8_t i = 0; i <= 2*N; i = i+gimbal_step){
            x[i/gimbal_step] = (i - N)*2;
        }

        float sin_x_values[n+1];
        float cos_x_values[n+1];

        for (int8_t i = 0; i < n+1; i++) {
            sin_x_values[i] = sinf(x[i]*DEG_TO_RAD);
            cos_x_values[i] = cosf(x[i]*DEG_TO_RAD);
        }

        float sin_mean = mean(sin_x_values, n+1);
        float cos_mean = mean(cos_x_values, n+1);
        float y_mean = mean(gimbal_probe_samples, n+1);

        float A1 = (dot_product(gimbal_probe_samples, sin_x_values, n+1) - y_mean * sin_mean * (n+1)) /
                    (dot_product(sin_x_values, sin_x_values, n+1) - sin_mean * sin_mean * (n+1));

        float B1 = (dot_product(gimbal_probe_samples, cos_x_values, n+1) - y_mean * cos_mean * (n+1)) /
                    (dot_product(cos_x_values, cos_x_values, n+1) - cos_mean * cos_mean * (n+1));

        float D = y_mean - A1 * sin_mean - B1 * cos_mean;

        float fitted_values[n+1];
        for(int i = 0; i < n+1; i++){
            fitted_values[i] = A1*sinf(x[i]*DEG_TO_RAD) + B1*cosf(x[i]*DEG_TO_RAD) + D;
        }
        
        float corr = correlation(gimbal_probe_samples, fitted_values, n+1);

        // Check the correlation coefficient. Alignment failed if corr is too low
        if(fabsf(corr) < 0.70){
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment failed: R = %f",corr);
            rotm_step.identity();
            copter.camera_mount.set_RotM_offset(rotm_step);
            gimbal_execute = false;
            return;
        }

        // Resultant angle offset
        float angle_offset = 0.5*atan2f(A1,B1)*RAD_TO_DEG;

        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment correction: %f deg",angle_offset);
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment success: R = %f",corr);

        // Send and apply resultant rotation matrix offset to the gimbal
        rotm_step.from_axis_angle(axis, ((double)angle_offset)*DEG_TO_RAD);
        copter.camera_mount.set_RotM_offset(rotm_step);

        gimbal_execute = false;
    }
}

float mean(const float* v, int n) {
    float sum = 0.0;
    for (int8_t i = 0; i < n; i++) {
        sum += v[i];
    }
    return sum / n;
}

float dot_product(const float* v1, const float* v2, int n) {
    double result = 0.0;
    for (int8_t i = 0; i < n; i++) {
        result += v1[i] * v2[i];
    }
    return result;
}

float standard_deviation(const float* v, float mean, int n) {
    float sum = 0.0;
    for (int8_t i = 0; i < n; i++) {
        float diff = v[i] - mean;
        sum += diff * diff;
    }
    return sqrtf(sum / (n - 1));
}

float correlation(const float* v1, const float* v2, int n) {
    float mean1 = mean(v1, n);
    float mean2 = mean(v2, n);
    float std_dev1 = standard_deviation(v1, mean1, n);
    float std_dev2 = standard_deviation(v2, mean2, n);
    float sum = 0.0;
    for (int8_t i = 0; i < n; i++) {
        sum += (v1[i] - mean1) * (v2[i] - mean2);
    }

    return sum / ((n - 1) * std_dev1 * std_dev2);
}

#endif

#ifdef USER_ARRC_SDR_LOOP
void Copter::user_ARRC_SDR_logger()
{
    // Read Power in dBm. Write sensors packet into the SD card
    // RFExplorer Power Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
    struct log_ARRC_SDR pkt_temp = {
        LOG_PACKET_HEADER_INIT(LOG_ARRC_SDR_MSG),
        local_timestamp         : copter.ARRC_SDR.get_local_timestamp(),      //Store time in microseconds
        boot_time               : copter.ARRC_SDR.get_boot_timestamp(),
        unix_time               : copter.ARRC_SDR.get_unix_timestamp(),
        pwr_c                   : copter.ARRC_SDR.get_pwr_c(),           //Store power in dBm
        pwr_x                   : copter.ARRC_SDR.get_pwr_x(),           //Store power in dBm
        phi_c                   : copter.ARRC_SDR.get_phi_c(),           //Store power in dBm
        phi_x                   : copter.ARRC_SDR.get_phi_x(),           //Store power in dBm
    };
    logger.WriteBlock(&pkt_temp, sizeof(pkt_temp));   //Send package to SD card
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    // Execution of the ARRC gimbal movement

    if(AP_HAL::millis() - switch_pause > 5000){
        if(alignment_done == false && gimbal_execute == false){
            gcs().send_text(MAV_SEVERITY_INFO, "Executing Gimbal alignment");
            memset(gimbal_probe_samples, 0, (gimbal_angle_span/gimbal_step + 1) * sizeof(float));
            gimbal_num_samples = 0;
            gimbal_iter = 0;
            gimbal_now = AP_HAL::millis();
            gimbal_execute = true;
            alignment_done = true;
        }
        else if(alignment_done == true && gimbal_execute == false){
            rotm_step.identity();
            copter.camera_mount.set_RotM_offset(rotm_step);
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal RotM offset cleared");
            alignment_done = false;
            gimbal_execute = false;
        }
        else{
            return;
        }
    }
    
    switch_pause = AP_HAL::millis();
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
