#include "Copter.h"
#include "arrc_polfit.h"

// ARRC gimbal alignment
// Rotates the probe about its boresight from -ALIGN_HALF_SPAN_DEG to +ALIGN_HALF_SPAN_DEG, averages the received
// power at each angle step and fits the polarization loss (see arrc_polfit.h) to find the rotation that aligns the
// probe polarization with the AUT. The result is saved in MNT1_ARRC_POFF; a failed or stopped alignment puts back
// the offset it started from. The power comes from the RF source in use when it starts: the SDR (co-polar power)
// while it is detected and an RF source, otherwise the LB5900. Aux switch 1 (RCx_OPTION 47) high starts it while
// MNT1_ARRC_GMODE is 2 (Hpol) or 3 (Vpol); low stops it and clears the offset
static constexpr int8_t ALIGN_HALF_SPAN_DEG = 32;           // scan from -32 to +32 degrees
static constexpr uint8_t ALIGN_STEP_DEG = 8;                // angle step
static constexpr uint8_t ALIGN_NUM_STEPS = 2 * ALIGN_HALF_SPAN_DEG / ALIGN_STEP_DEG + 1;
static constexpr uint32_t ALIGN_INIT_WAIT_MS = 4000;        // waiting time for the gimbal to reach the first angle
static constexpr uint32_t ALIGN_STEP_WAIT_MS = 3000;        // waiting time for the gimbal to reach each following angle
static constexpr uint32_t ALIGN_SAMPLE_MS = 10000;          // power sampling time at each angle step
static constexpr uint32_t ALIGN_SENSOR_TIMEOUT_MS = 1000;   // abort if the power sensor has no new sample for this long
static constexpr uint16_t ALIGN_MIN_SAMPLES = 5;            // minimum number of power samples at each angle step
static constexpr float ALIGN_MAX_SIGMA_DEG = 2.0f;          // largest standard error of the fitted offset accepted
static_assert((2 * ALIGN_HALF_SPAN_DEG) % ALIGN_STEP_DEG == 0, "the angle span must be a multiple of the step");

enum class AlignState : uint8_t {
    IDLE,
    MOVING,     // waiting for the gimbal to reach the angle step
    SAMPLING,   // averaging the power at the angle step
};

static struct {
    AlignState state;
    bool sdr;                           // power from the SDR, else from the LB5900
    uint8_t step;                       // angle step index
    uint32_t state_start_ms;            // system time the state started
    uint32_t last_sample_ms;            // system time of the last new power sample
    uint32_t last_sample_count;         // power sensor sample count at the last power sample
    uint32_t num_samples;               // number of power samples at this angle step
    double power_sum_mw;                // sum of the power samples at this angle step in mW
    float start_offset_deg;             // MNT1_ARRC_POFF when the alignment started
    float angle_deg[ALIGN_NUM_STEPS];   // probe rotation at each angle step
    float power_mw[ALIGN_NUM_STEPS];    // average power at each angle step in mW
} gimbal_align;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //ARRC Gimbal params init
    gimbal_align.state = AlignState::IDLE;
}
#endif

bool Copter::arrc_align_sample(bool sdr, uint32_t &count, float &power_dbm) const
{
#if AP_ARRC_SDR_ENABLED
    if (sdr) {
        return ARRC_SDR.get_copol(power_dbm, count);
    }
#endif
    AP_ARRC_LB5900::Sample sample;
    const bool ok = ARRC_LB5900.get_sample(sample) && ARRC_LB5900.healthy();
    count = sample.count;
    power_dbm = sample.power_dbm;
    return ok;
}

#ifdef USER_GIMBAL_LOOP
void Copter::user_ARRC_gimbal()
{
    if (gimbal_align.state == AlignState::IDLE) {
        return;
    }

    // stop the alignment and put back the offset it started from (the scan does not save its angles)
    auto abort_alignment = [this](const char *reason) {
        camera_mount.set_ARRC_pol_offset(gimbal_align.start_offset_deg, false);
        gimbal_align.state = AlignState::IDLE;
        gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment failed: %s", reason);
    };

    const uint8_t gimbal_mode = camera_mount.get_ARRC_gimbal_mode();
    if (gimbal_mode != 2 && gimbal_mode != 3) {
        abort_alignment("gimbal mode changed");
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
        float power_dbm;
        IGNORE_RETURN(arrc_align_sample(gimbal_align.sdr, gimbal_align.last_sample_count, power_dbm));
        gimbal_align.state = AlignState::SAMPLING;
        gimbal_align.state_start_ms = now_ms;
        gimbal_align.last_sample_ms = now_ms;
        gimbal_align.num_samples = 0;
        gimbal_align.power_sum_mw = 0;
        break;
    }

    case AlignState::SAMPLING: {
        // Collect new power measurements in linear units (mW) during the sampling time
        uint32_t sample_count;
        float power_dbm;
        const bool valid = arrc_align_sample(gimbal_align.sdr, sample_count, power_dbm);
        if (sample_count != gimbal_align.last_sample_count) {
            gimbal_align.last_sample_count = sample_count;
            if (valid) {
                gimbal_align.last_sample_ms = now_ms;
                gimbal_align.power_sum_mw += powf(10.0f, 0.1f * power_dbm);
                gimbal_align.num_samples++;
            }
        }
        if (now_ms - gimbal_align.last_sample_ms > ALIGN_SENSOR_TIMEOUT_MS) {
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

        // All angle steps done: fit the polarization loss and apply the resultant offset if it is well determined
        float angle_offset = 0;
        float corr = 0;
        float sigma = 0;
        if (!ARRC_fit_polarization(gimbal_align.angle_deg, gimbal_align.power_mw, ALIGN_NUM_STEPS, angle_offset, corr,
                                   sigma)) {
            abort_alignment("no power variation");
            return;
        }
        if (sigma > ALIGN_MAX_SIGMA_DEG) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment: %.1f deg +-%.1f, R %.3f", angle_offset, sigma, corr);
            abort_alignment("offset too uncertain");
            return;
        }
        gimbal_align.state = AlignState::IDLE;
        camera_mount.set_ARRC_pol_offset(angle_offset, true);
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment correction: %.2f deg", angle_offset);
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment success: +-%.2f deg, R = %.3f", sigma, corr);
        break;
    }
    }
}
#endif

// USR_SDR_OPT bits
static constexpr int16_t SDR_OPT_TX_STOP_DISARM = (1U << 0);
static constexpr int16_t SDR_OPT_TX_STOP_FAILSAFE = (1U << 1);
static constexpr int16_t SDR_OPT_REQUIRE_MEAS = (1U << 2);
static constexpr int16_t SDR_OPT_REQUIRE_LB5900 = (1U << 3);
static constexpr uint32_t SDR_ARMING_MEAS_MAX_AGE_MS = 2000;

Copter::ARRCRFSource Copter::arrc_rf_source() const
{
    const int8_t src = g2.user_parameters.get_rf_src();
    if (src < 0 || src > int8_t(ARRCRFSource::BOTH)) {
        return ARRCRFSource::AUTO;
    }
    return ARRCRFSource(src);
}

// true if the LB5900 power is logged and streamed (CASS_SENSOR_RAW)
bool Copter::arrc_lb5900_streaming() const
{
    return ARRC_LB5900.enabled() && arrc_rf_source() != ARRCRFSource::SDR;
}

// true if the SDR measurement is streamed (ARRC_SENSOR_RAW)
bool Copter::arrc_sdr_streaming() const
{
#if AP_ARRC_SDR_ENABLED
    return arrc_rf_source() != ARRCRFSource::LB5900;
#else
    return false;
#endif
}

bool Copter::arrc_arming_checks(size_t buflen, char *buffer) const
{
    const int16_t options = g2.user_parameters.get_sdr_options();
#if AP_ARRC_SDR_ENABLED
    if (options & SDR_OPT_REQUIRE_MEAS) {
        if (!ARRC_SDR.detected()) {
            hal.util->snprintf(buffer, buflen, "SDR: companion or SDR not detected");
            return false;
        }
        if (!ARRC_SDR.meas_fresh(SDR_ARMING_MEAS_MAX_AGE_MS)) {
            hal.util->snprintf(buffer, buflen, "SDR: no measurements");
            return false;
        }
    }
#endif
    if ((options & SDR_OPT_REQUIRE_LB5900) && arrc_lb5900_streaming() && !ARRC_LB5900.healthy()) {
        hal.util->snprintf(buffer, buflen, "LB5900: no readings");
        return false;
    }
    return true;
}

#if defined(USER_ARRC_SDR_LOOP) && AP_ARRC_SDR_ENABLED
void Copter::user_ARRC_SDR_logger()
{
    const uint32_t timeout_ms = constrain_float(g2.user_parameters.get_sdr_timeout(), 1.0f, 10.0f) * 1000;
    ARRC_SDR.update(timeout_ms);

    // AUTO: once the companion computer and its SDR are detected, the LB5900 is disabled until reboot
    if (arrc_rf_source() == ARRCRFSource::AUTO && !arrc_lb5900_latched_off && ARRC_SDR.detected()) {
        ARRC_LB5900.set_enabled(false);
        arrc_lb5900_latched_off = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RF source: SDR (LB5900 disabled)");
#if HAL_LOGGING_ENABLED
// @LoggerMessage: RFSR
// @Description: ARRC RF power source change
// @Field: TimeUS: Time since system startup
// @Field: Src: RF source parameter (USR_RF_SRC)
// @Field: LB: LB5900 enabled
// @Field: SDR: Companion and SDR detected
        logger.Write("RFSR", "TimeUS,Src,LB,SDR", "s---", "F---", "QBBB",
                     AP_HAL::micros64(),
                     uint8_t(arrc_rf_source()),
                     uint8_t(ARRC_LB5900.enabled()),
                     uint8_t(ARRC_SDR.detected()));
#endif
    }

    // stop SDR transmission on disarm or failsafe (the companion also stops on its own interlocks)
    const int16_t options = g2.user_parameters.get_sdr_options();
    const bool armed = motors->armed();
    const bool in_failsafe = any_failsafe_triggered();
    const bool disarmed_now = (options & SDR_OPT_TX_STOP_DISARM) && arrc_was_armed && !armed;
    const bool failsafe_now = (options & SDR_OPT_TX_STOP_FAILSAFE) && in_failsafe && !arrc_was_failsafe;
    if ((disarmed_now || failsafe_now) && ARRC_SDR.detected()) {
        if (ARRC_SDR.send_tx_stop()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SDR: TX stop sent (%s)", failsafe_now ? "failsafe" : "disarm");
        }
    }
    arrc_was_armed = armed;
    arrc_was_failsafe = in_failsafe;
}
#endif

#ifdef USER_ARRCLB5900_LOOP
void Copter::user_LB5900_logger()
{
    if (!arrc_lb5900_streaming()) {
        return;
    }
    arrc_lb5900_update();
}
#endif

// LB5900 at 50 Hz: offer each new reading to the channels streaming CASS_SENSOR_RAW, log it as LB59, and
// report the sensor state to the GCS
void Copter::arrc_lb5900_update()
{
    AP_ARRC_LB5900::Sample sample;
    const bool have_sample = ARRC_LB5900.get_sample(sample);
    const uint32_t now_ms = AP_HAL::millis();

    if (have_sample && sample.count != arrc_lb.pushed_count) {
        arrc_lb.pushed_count = sample.count;
        for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
            uint16_t interval_ms;
            if (gcs().chan(c)->get_ap_message_interval(MSG_ARRC_SDR, interval_ms) && interval_ms > 0) {
                gcs().chan(c)->send_message(MSG_ARRC_SDR);
            }
        }
    }

    // LB5900 Power Data Logger: each reading once, at the time it was read; while the sensor is not healthy a
    // record with Hth 0 once a second
// @LoggerMessage: LB59
// @Description: ARRC LB5900 RF power sensor reading
// @Field: TimeUS: Time the reading was taken
// @Field: Hth: Sensor healthy
// @Field: Pwr: Power (NaN while the sensor is not healthy)
// @Field: Cnt: Readings taken since boot
    if (have_sample && sample.count != arrc_lb.logged_count) {
        arrc_lb.logged_count = sample.count;
        const struct log_LB5900 pkt {
            LOG_PACKET_HEADER_INIT(LOG_ARRC_LB59_MSG),
            time_stamp  : uint64_t(sample.time_ms) * 1000U,
            healthy     : 1,
            power       : sample.power_dbm,
            count       : sample.count,
        };
        logger.WriteBlock(&pkt, sizeof(pkt));
    } else if (!ARRC_LB5900.healthy() && now_ms - arrc_lb.unhealthy_log_ms >= 1000) {
        arrc_lb.unhealthy_log_ms = now_ms;
        const struct log_LB5900 pkt {
            LOG_PACKET_HEADER_INIT(LOG_ARRC_LB59_MSG),
            time_stamp  : AP_HAL::micros64(),
            healthy     : 0,
            power       : NaNf,
            count       : sample.count,
        };
        logger.WriteBlock(&pkt, sizeof(pkt));
    }

    // sensor state for the GCS
    const AP_ARRC_LB5900::State state = ARRC_LB5900.state();
    if (state != arrc_lb.reported_state) {
        if (state == AP_ARRC_LB5900::State::RUNNING) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LB5900: %s",
                          arrc_lb.lost ? "reconnected" : (arrc_lb.not_found_reported ? "found" : "OK"));
            arrc_lb.lost = false;
        } else if (state == AP_ARRC_LB5900::State::CONFIGURING &&
                   arrc_lb.reported_state == AP_ARRC_LB5900::State::RUNNING) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LB5900: no readings, reconnecting");
            arrc_lb.lost = true;
        }
        arrc_lb.reported_state = state;
    }
    if (state == AP_ARRC_LB5900::State::CONFIGURING && !ARRC_LB5900.ever_configured() &&
        !arrc_lb.not_found_reported && now_ms - arrc_lb.init_ms > 5000) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LB5900: not found (I2C bus %u, address %u)",
                      unsigned(g2.user_parameters.get_lb5900_busId()),
                      unsigned(g2.user_parameters.get_lb5900_address()));
        arrc_lb.not_found_reported = true;
    }
}

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
    // ARRC gimbal alignment: high starts it, low stops it (if running) and clears the polarization offset
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH: {
        if (gimbal_align.state != AlignState::IDLE) {
            return;
        }
        const uint8_t gimbal_mode = camera_mount.get_ARRC_gimbal_mode();
        if (gimbal_mode != 2 && gimbal_mode != 3) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment needs MNT1_ARRC_GMODE 2 or 3");
            return;
        }
        // the RF source in use: the SDR while detected, otherwise the LB5900
        uint32_t count;
        float power_dbm;
        bool sdr = false;
#if AP_ARRC_SDR_ENABLED
        sdr = arrc_sdr_streaming() && ARRC_SDR.detected() && ARRC_SDR.meas_fresh(1000) &&
              arrc_align_sample(true, count, power_dbm);
#endif
        if (!sdr && !(arrc_lb5900_streaming() && arrc_align_sample(false, count, power_dbm))) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal alignment needs an RF power reading");
            return;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Executing Gimbal alignment (%s)", sdr ? "SDR" : "LB5900");
        for (uint8_t i = 0; i < ALIGN_NUM_STEPS; i++) {
            gimbal_align.angle_deg[i] = -ALIGN_HALF_SPAN_DEG + i * ALIGN_STEP_DEG;
            gimbal_align.power_mw[i] = 0;
        }
        gimbal_align.sdr = sdr;
        gimbal_align.start_offset_deg = camera_mount.get_ARRC_pol_offset();
        gimbal_align.step = 0;
        gimbal_align.state_start_ms = AP_HAL::millis();
        gimbal_align.state = AlignState::MOVING;
        camera_mount.set_ARRC_pol_offset(gimbal_align.angle_deg[0], false);
        break;
    }
    case RC_Channel::AuxSwitchPos::LOW:
        if (gimbal_align.state != AlignState::IDLE) {
            gimbal_align.state = AlignState::IDLE;
            gcs().send_text(MAV_SEVERITY_INFO, "Gimbal alignment stopped");
        }
        camera_mount.set_ARRC_pol_offset(0.0f, true);
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal polarization offset cleared");
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    }
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
