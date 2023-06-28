#include "Copter.h"

//ARRC Gimbal function global params
uint32_t gimbal_now;
uint32_t switch_pause;
bool gimbal_execute;
uint8_t gimbal_iter;
const uint8_t gimbal_angle_span = 30;        // Must be an even number
const uint8_t gimbal_step = 5;              // Angle steps
const uint16_t gimbal_init_wait = 4000;      // Gimbal initial waiting time
const uint16_t gimbal_wait = 3000;           // Waiting time while gimbal is rotating
const uint16_t gimbal_sample_time = 2000;    // Sampling time at each angle step in milliseconds
float gimbal_probe_samples[gimbal_angle_span/gimbal_step + 1];
uint8_t gimbal_num_samples;
bool alignment_done;
Matrix3d rotm_step;

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
                gimbal_probe_samples[gimbal_iter/gimbal_step] = gimbal_probe_samples[gimbal_iter/gimbal_step] + copter.ARRC_LB680A.get_pwr();
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

        // // Simulated data points for code test and debugging
        // if (kek == 0){
        //     float newValues[] = {-10.6525, -10.2897, -10.0700, -10.0001, -10.0821, -10.3136, -10.6874}; // 0.2 deg offset, measured: -0.197742
        //     for(int i = 0; i < 7; i++) {
        //         gimbal_probe_samples[i] = newValues[i];
        //     }
        //     kek = kek + 1;
        // }
        // else if (kek == 1){
        //     float newValues[] = {-10.5450, -10.2185, -10.0373, -10.0069, -10.1281, -10.3975, -10.8066}; // 1.5 deg offset, measured: -1.484070
        //     for(int i = 0; i < 7; i++) {
        //         gimbal_probe_samples[i] = newValues[i];
        //     }
        //     kek = kek + 1;
        // }
        // else{
        //     float newValues[] = {-10.0760, -10.0000, -10.0760, -10.3015, -10.6699, -11.1698, -11.7861}; // 10 deg offset, measured: -10.30861
        //     for(int i = 0; i < 7; i++) {
        //         gimbal_probe_samples[i] = newValues[i];
        //     }
        //     kek = 0;
        // }

        int8_t i,j,k;
        int8_t n = gimbal_angle_span/gimbal_step;

        // Create array of angle steps corresponding to each power measurement
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
                X[i]=X[i]+powf(x[j],i);          //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
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
            Y[i]=Y[i]+powf(x[j],i)*gimbal_probe_samples[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
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
                if (j!=i)                       //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
                    a[i]=a[i]-B[i][j]*a[j];
            a[i]=a[i]/B[i][i];                  //now finally divide the rhs by the coefficient of the variable to be calculated
        }

        // Check if we got a local maxima. Otherwise, the alignment failed
        if(a[2] > 0 || is_zero(a[2])){
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment failed: No Maxima");
            rotm_step.identity();
            copter.camera_mount.set_RotM_offset(rotm_step);
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
            rotm_step.identity();
            copter.camera_mount.set_RotM_offset(rotm_step);
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

#ifdef USER_LB680A_LOOP
void Copter::user_ARRC_LB680A_logger()
{
    // Read Power in dBm. Write sensors packet into the SD card
    // RFExplorer Power Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
    struct log_LB680A pkt_temp = {
        LOG_PACKET_HEADER_INIT(LOG_LB680A_MSG),
        time_stamp              : copter.ARRC_LB680A.get_timestamp(),      //Store time in microseconds
        pwr                     : copter.ARRC_LB680A.get_pwr(),           //Store power in dBm
        pkpwr                   : copter.ARRC_LB680A.get_pkpwr(),          //Store peak power in dBm
        avgpwr                  : copter.ARRC_LB680A.get_avgpwr(),          //Store average power dBm
        dcyc                    : copter.ARRC_LB680A.get_dcyc(),          //Store duty cycle
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
