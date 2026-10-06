#include "Copter.h"
#include "arrc_polfit.h"

// ARRC gimbal alignment
// Rotates the probe about its boresight from -ALIGN_HALF_SPAN_DEG to +ALIGN_HALF_SPAN_DEG, averages the received
// power at each angle step and fits the polarization loss (see arrc_polfit.h) to find the rotation that aligns the
// probe polarization with the AUT. The result is saved in MNT1_ARRC_POFF. Runs from aux switch 1 (CHx_OPT = 47)
// while MNT1_ARRC_GMODE is 2 (Hpol) or 3 (Vpol)
static constexpr int8_t ALIGN_HALF_SPAN_DEG = 32;           // scan from -32 to +32 degrees
static constexpr uint8_t ALIGN_STEP_DEG = 8;                // angle step
static constexpr uint8_t ALIGN_NUM_STEPS = 2 * ALIGN_HALF_SPAN_DEG / ALIGN_STEP_DEG + 1;
static constexpr uint32_t ALIGN_INIT_WAIT_MS = 4000;        // waiting time for the gimbal to reach the first angle
static constexpr uint32_t ALIGN_STEP_WAIT_MS = 3000;        // waiting time for the gimbal to reach each following angle
static constexpr uint32_t ALIGN_SAMPLE_MS = 10000;          // power sampling time at each angle step
static constexpr uint32_t ALIGN_SENSOR_TIMEOUT_MS = 1000;   // abort if the power sensor has no new sample for this long
static constexpr uint16_t ALIGN_MIN_SAMPLES = 5;            // minimum number of power samples at each angle step
static constexpr float ALIGN_MIN_R = 0.7f;                  // minimum correlation coefficient of the fit
static_assert((2 * ALIGN_HALF_SPAN_DEG) % ALIGN_STEP_DEG == 0, "the angle span must be a multiple of the step");

enum class AlignState : uint8_t {
    IDLE,
    MOVING,     // waiting for the gimbal to reach the angle step
    SAMPLING,   // averaging the power at the angle step
};

static struct {
    AlignState state;
    uint8_t step;                       // angle step index
    uint32_t state_start_ms;            // system time the state started
    uint32_t last_sample_ms;            // system time of the last new power sample
    uint32_t last_sample_count;         // power sensor sample count at the last power sample
    uint32_t num_samples;               // number of power samples at this angle step
    double power_sum_mw;                // sum of the power samples at this angle step in mW
    float angle_deg[ALIGN_NUM_STEPS];   // probe rotation at each angle step
    float power_mw[ALIGN_NUM_STEPS];    // average power at each angle step in mW
} gimbal_align;

static bool alignment_done;             // true if the next switch change clears the offset instead of aligning
static uint32_t switch_pause_ms;        // system time of the last switch change

//LB5900 global params
uint32_t LB_now;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //ARRC Gimbal params init
    switch_pause_ms = AP_HAL::millis();
    gimbal_align.state = AlignState::IDLE;
    alignment_done = true;

    //LB5900 initialize
    LB_now = AP_HAL::millis();
}
#endif

#ifdef USER_GIMBAL_LOOP
void Copter::user_ARRC_gimbal()
{
    if (gimbal_align.state == AlignState::IDLE) {
        return;
    }

    // stop the alignment and remove the offset
    auto abort_alignment = [this](const char *reason) {
        camera_mount.set_ARRC_pol_offset(0.0f, true);
        gimbal_align.state = AlignState::IDLE;
        gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment failed: %s", reason);
    };

    const uint8_t gimbal_mode = camera_mount.get_ARRC_gimbal_mode();
    if (gimbal_mode != 2 && gimbal_mode != 3) {
        abort_alignment("gimbal mode changed");
        return;
    }
    if (!ARRC_LB5900.healthy()) {
        abort_alignment("power sensor unhealthy");
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    switch (gimbal_align.state) {
    case AlignState::IDLE:
        break;

    case AlignState::MOVING: {
        // Wait for the gimbal to complete rotating
        const uint32_t wait_ms = (gimbal_align.step == 0) ? ALIGN_INIT_WAIT_MS : ALIGN_STEP_WAIT_MS;
        if (now_ms - gimbal_align.state_start_ms < wait_ms) {
            return;
        }
        gimbal_align.state = AlignState::SAMPLING;
        gimbal_align.state_start_ms = now_ms;
        gimbal_align.last_sample_ms = now_ms;
        gimbal_align.last_sample_count = ARRC_LB5900.sample_count();
        gimbal_align.num_samples = 0;
        gimbal_align.power_sum_mw = 0;
        break;
    }

    case AlignState::SAMPLING: {
        // Collect new power measurements in linear units (mW) during the sampling time
        const uint32_t sample_count = ARRC_LB5900.sample_count();
        if (sample_count != gimbal_align.last_sample_count) {
            gimbal_align.last_sample_count = sample_count;
            gimbal_align.last_sample_ms = now_ms;
            gimbal_align.power_sum_mw += powf(10.0f, 0.1f * ARRC_LB5900.power_measure());
            gimbal_align.num_samples++;
        } else if (now_ms - gimbal_align.last_sample_ms > ALIGN_SENSOR_TIMEOUT_MS) {
            abort_alignment("no power samples");
            return;
        }
        if (now_ms - gimbal_align.state_start_ms < ALIGN_SAMPLE_MS) {
            return;
        }
        if (gimbal_align.num_samples < ALIGN_MIN_SAMPLES) {
            abort_alignment("too few power samples");
            return;
        }

        // Average the collected power measurements
        gimbal_align.power_mw[gimbal_align.step] = gimbal_align.power_sum_mw / gimbal_align.num_samples;

        // Rotate the gimbal to the next angle step
        gimbal_align.step++;
        if (gimbal_align.step < ALIGN_NUM_STEPS) {
            camera_mount.set_ARRC_pol_offset(gimbal_align.angle_deg[gimbal_align.step], false);
            gimbal_align.state = AlignState::MOVING;
            gimbal_align.state_start_ms = now_ms;
            return;
        }

        // All angle steps done: fit the polarization loss and apply the resultant offset
        gimbal_align.state = AlignState::IDLE;
        float angle_offset = 0;
        float corr = 0;
        if (!ARRC_fit_polarization(gimbal_align.angle_deg, gimbal_align.power_mw, ALIGN_NUM_STEPS, angle_offset, corr)) {
            camera_mount.set_ARRC_pol_offset(0.0f, true);
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment failed: no power variation");
            return;
        }
        // Check the correlation coefficient. Alignment failed if corr is too low
        if (corr < ALIGN_MIN_R) {
            camera_mount.set_ARRC_pol_offset(0.0f, true);
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment failed: R = %.3f", corr);
            return;
        }
        camera_mount.set_ARRC_pol_offset(angle_offset, true);
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment correction: %.2f deg", angle_offset);
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment success: R = %.3f", corr);
        break;
    }
    }
}
#endif

#ifdef USER_ARRC_SDR_LOOP
void Copter::user_ARRC_SDR_logger()
{
}
#endif

#ifdef USER_ARRCLB5900_LOOP
void Copter::user_LB5900_logger()
{
    // Read Power in dBm. Write sensors packet into the SD card
    // LB5900 Power Data Logger ///////////////////////////////////////////////////////////////////////////////////////////
    struct log_LB5900 pkt_temp = {
        LOG_PACKET_HEADER_INIT(LOG_ARRC_LB59_MSG),
        time_stamp              : AP_HAL::micros64(),                   //Store time in microseconds
        healthy                 : copter.ARRC_LB5900.healthy(),         //Store sensor health
        power                   : copter.ARRC_LB5900.power_measure(),   //Store power in dBm
    };
    logger.WriteBlock(&pkt_temp, sizeof(pkt_temp));   //Send package to SD card

    // Print desired params for Debugging
    // if (AP_HAL::millis() - LB_now > 3000){

    //     const char* (mrate[1])[4] = 
    //     {
    //         "NORMAL",   // 20 readings per sec
    //         "DOUBLE",   // 40 readings per sec
    //         "FAST",     // 110 readings per sec (disallows average count)
    //         "SUPER"     // 110 readings per sec (allows average count)
    //     };

    //     gcs().send_text(MAV_SEVERITY_INFO,"LB init: %d",(uint8_t)copter.ARRC_LB5900.has_init());
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
    // Execution of the ARRC gimbal alignment: switch changes alternately clear the polarization offset and start the
    // alignment. Changes within 5 seconds of the previous one, or while the alignment runs, are ignored

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - switch_pause_ms > 5000) {
        if (gimbal_align.state != AlignState::IDLE) {
            return;
        }
        if (!alignment_done) {
            const uint8_t gimbal_mode = camera_mount.get_ARRC_gimbal_mode();
            if (gimbal_mode != 2 && gimbal_mode != 3) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment needs MNT1_ARRC_GMODE 2 or 3");
            } else if (!ARRC_LB5900.healthy()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment needs a healthy power sensor");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Executing Gimbal alignment");
                for (uint8_t i = 0; i < ALIGN_NUM_STEPS; i++) {
                    gimbal_align.angle_deg[i] = -ALIGN_HALF_SPAN_DEG + i * ALIGN_STEP_DEG;
                    gimbal_align.power_mw[i] = 0;
                }
                gimbal_align.step = 0;
                gimbal_align.state_start_ms = now_ms;
                gimbal_align.state = AlignState::MOVING;
                camera_mount.set_ARRC_pol_offset(gimbal_align.angle_deg[0], false);
                alignment_done = true;
            }
        } else {
            camera_mount.set_ARRC_pol_offset(0.0f, true);
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal polarization offset cleared");
            alignment_done = false;
        }
    }

    switch_pause_ms = now_ms;
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
