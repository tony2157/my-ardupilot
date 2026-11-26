#include "Copter.h"
#include "arrc_polfit.h"

/*
  ARRC transmit differential phase sweep (MAV_CMD_ARRC_DPHASE)

  The sweep switches the gimbal to parallel planes (MNT1_ARRC_GMODE 1, not saved) and back to its mode at the end:
  the probe faces along the AUT boresight (MNT1_ARRC_AZTH, pitch -MNT1_ARRC_ELEV) and the gimbal turns it about
  that boresight through the angles -span..+span, and back unless one way (AP_Mount::set_ARRC_parallel_rotation,
  not stored; MNT1_ARRC_POFF is not used). A mode set by someone else during the sweep stops it and is kept. At each angle it waits
  for the gimbal to settle, averages the RF power in mW and the polarization error of the gimbal's measured
  attitude about the commanded boresight; the fit uses that measured angle. The fit (arrc_polfit.h) of a
  beam that carries H and V at once gives abs(beta), ZDR_tx and the swing. Every exit sets the rotation back to 0.

  Fit: three terms, or three terms plus sin(4 psi) and cos(4 psi) for a probe whose response changes with its roll
  (the default). The command chooses (ARRC_DPHASE_OPTIONS_FIT_3TERM, FIT_4PSI), else USR_TXDIFFX_OPT. The 4 psi terms
  need a wide sweep to be told from the others: below +-45 deg or 7 angles the three-term fit is used. The flag
  ARRC_DPHASE_FLAGS_FIT_4PSI says which fit the result is from.
  Reported in ARRC_DPHASE_STATUS and ARRC_DPHASE_STEP, the ARDP and ARDR logs and GCS text.

  Convention: the angle turns the probe from the AUT y axis, to the right seen from behind the radar looking along
  the beam, toward up. beta is given with H to the left and V up seen from behind the radar, so angle 0 is along -H
  (h_sign -1); with the opposite polarity of H or V it is 180 - beta.

  Probe orientation: the fit needs the angle of the probe's polarization, and the gimbal only knows its own roll.
  The command says how the probe is mounted (ARRC_DPHASE_OPTIONS_PROBE_H or PROBE_V: its polarization at angle 0).
  A vertical probe is 90 deg further round, so the fit takes the angle plus 90 deg and beta and ZDR_tx come out for
  the probe as mounted. With neither option the result assumes a horizontal probe, as before, and both candidates
  are printed.
*/

#if HAL_MOUNT_ENABLED

static constexpr uint8_t DPH_MAX_STEPS = 64;            // angles in a run, the way back included
static constexpr float DPH_MAX_SPAN_DEG = 60;           // the ARRC roll limit (AP_Mount_Backend, T7 roll travel)
static constexpr float DPH_4PSI_MIN_SPAN_DEG = 45;      // the 4 psi terms need at least this span
static constexpr uint8_t DPH_4PSI_MIN_ANGLES = 7;       // and this many angles one way
static constexpr uint32_t DPH_FIRST_EXTRA_MS = 2000;    // extra settling for the first move
static constexpr uint32_t DPH_ARRIVE_TIMEOUT_MS = 5000; // extra wait for the gimbal to arrive after settling
static constexpr float DPH_ARRIVE_TOL_DEG = 3;          // measured polarization error accepted to start averaging
static constexpr uint32_t DPH_SENSOR_TIMEOUT_MS = 1000; // stop if the RF source has no new reading for this long
static constexpr uint32_t DPH_MODE_GRACE_MS = 1000;     // after the start, time for the parallel-planes solution
static constexpr uint16_t DPH_MIN_SAMPLES = 5;          // valid readings needed at each angle
static constexpr int8_t DPH_H_SIGN = -1;                // angle 0 is along -H (see above)
// readings left out of the average: clipped, samples lost, another waveform averaged in, or transmitting
static constexpr uint16_t DPH_REJECT_FLAGS = ARRC_MEAS_FLAGS_SATURATED | ARRC_MEAS_FLAGS_OVERRUN |
                                             ARRC_MEAS_FLAGS_WF_MISMATCH | ARRC_MEAS_FLAGS_TX_ACTIVE;

static struct {
    uint8_t state;          // ARRC_DPHASE_STATE
    uint8_t reason;         // ARRC_DPHASE_REASON
    uint16_t run_id;
    uint16_t flags;         // ARRC_DPHASE_FLAGS
    bool sdr;               // power from the SDR, else the LB5900
    // settings of the run
    float span;
    float step;
    float settle;
    uint8_t dwell;
    uint8_t n_one_way;      // angles from -span to +span
    uint8_t n_steps;        // angles in the run
    // progress
    uint8_t idx;            // current angle
    uint8_t n_done;         // angles averaged
    uint8_t start_gmode;    // MNT1_ARRC_GMODE before the run, put back at its end
    float probe_deg;        // angle of the probe's polarization at gimbal angle 0: 0 horizontal, 90 vertical
    bool fit_4psi;          // fit with the 4 psi terms asked for (command, else USR_TXDIFFX_OPT)
    // the fitted terms of P = D + A sin(2x) + B cos(2x) + C4 sin(4x) + E4 cos(4x), x = gimbal angle + probe_deg,
    // in units of the largest averaged power (p_max_dbm); C4 and E4 are NaN for the three-term fit
    float term_d, term_a, term_b, c4, e4, p_max_dbm;
    uint32_t run_start_ms;  // system time the run started
    uint32_t state_ms;      // system time the state started
    uint32_t last_sample_ms;
    uint32_t last_count;    // RF source reading count at the last reading
    // averaging at the current angle
    double sum_mw;          // readings in mW, weighted by the pulses they average
    double sum_w;
    double sum_db;          // readings in dB, for their spread
    double sum_db2;
    uint16_t n;
    uint16_t n_rej;
    uint16_t meas_flags;
    double sum_pol;         // polarization error of the measured attitude
    uint16_t n_pol;
    struct Step {
        uint32_t time_ms;
        float psi;
        float psi_meas;
        float power_dbm;
        float power_sd;
        uint16_t n;
        uint16_t n_rej;
        uint16_t meas_flags;
    } steps[DPH_MAX_STEPS];
    ARRC_TxDiffPhase result;
    float fit_r;
} dph;

// probe angle of step i: -span to +span by step (the last one at +span), then back
static float dph_angle(uint8_t i)
{
    if (i >= dph.n_one_way) {
        i = 2 * (dph.n_one_way - 1) - i;
    }
    if (i == dph.n_one_way - 1) {
        return dph.span;
    }
    return -dph.span + i * dph.step;
}

static const char *dph_reason_text(uint8_t reason)
{
    switch (reason) {
    case ARRC_DPHASE_REASON_COMMAND:
        return "stopped by command";
    case ARRC_DPHASE_REASON_GIMBAL_MODE:
        return "MNT1_ARRC_GMODE changed/invalid";
    case ARRC_DPHASE_REASON_NO_SOLUTION:
        return "no parallel-planes pointing";
    case ARRC_DPHASE_REASON_LIMIT:
        return "gimbal roll or pitch limit";
    case ARRC_DPHASE_REASON_RF_SOURCE:
        return "no RF power reading";
    case ARRC_DPHASE_REASON_FEW_SAMPLES:
        return "too few power readings";
    case ARRC_DPHASE_REASON_VEHICLE:
        return "flight mode or failsafe";
    case ARRC_DPHASE_REASON_ALIGNMENT:
        return "gimbal alignment running";
    case ARRC_DPHASE_REASON_FIT:
        return "fit failed";
    case ARRC_DPHASE_REASON_SETTINGS:
        return "settings out of range";
    case ARRC_DPHASE_REASON_GIMBAL:
        return "gimbal did not reach the angle";
    case ARRC_DPHASE_REASON_BUSY:
        return "already running";
    default:
        return "";
    }
}

// queue a message to the links that stream ARRC_DPHASE_STATUS
static void dph_push(ap_message id)
{
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        GCS_MAVLINK *link = gcs().chan(c);
        uint16_t interval_ms;
        if (link != nullptr && link->get_ap_message_interval(MSG_ARRC_DPHASE_STATUS, interval_ms) && interval_ms > 0) {
            link->send_message(id);
        }
    }
}

bool Copter::arrc_dphase_running() const
{
    return dph.state == ARRC_DPHASE_STATE_MOVING || dph.state == ARRC_DPHASE_STATE_SAMPLING;
}

// the vehicle allows a sweep: disarmed, or armed in Loiter, PosHold, Guided or Auto without a failsafe
bool Copter::arrc_dphase_vehicle_ok() const
{
    if (any_failsafe_triggered()) {
        return false;
    }
    if (!motors->armed()) {
        return true;
    }
    switch (flightmode->mode_number()) {
    case Mode::Number::LOITER:
    case Mode::Number::POSHOLD:
    case Mode::Number::GUIDED:
    case Mode::Number::AUTO:
        return true;
    default:
        return false;
    }
}

// the latest LB5900 reading: count changes with each reading. flags 0 and weight 1 (the SDR's measurements are
// summed as they arrive, AP_ARRC_SDR::accum())
bool Copter::arrc_dphase_sample(uint32_t &count, float &power_dbm, uint16_t &flags, float &weight) const
{
    flags = 0;
    weight = 1;
    return arrc_align_sample(false, count, power_dbm);
}

// the gimbal mode from before the run, unless someone set another one than parallel planes during it
void Copter::arrc_dphase_restore_gmode()
{
    if (dph.start_gmode != ARRC_GIMBAL_MODE_PARALLEL_PLANES &&
        camera_mount.get_ARRC_gimbal_mode() == ARRC_GIMBAL_MODE_PARALLEL_PLANES) {
        camera_mount.set_ARRC_gimbal_mode(dph.start_gmode);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: MNT1_ARRC_GMODE back to %u", unsigned(dph.start_gmode));
    }
}

void Copter::arrc_dphase_stop(uint8_t reason)
{
#if AP_ARRC_SDR_ENABLED
    ARRC_SDR.accum_stop();
#endif
    camera_mount.set_ARRC_parallel_rotation(0);
    arrc_dphase_restore_gmode();
    dph.state = ARRC_DPHASE_STATE_STOPPED;
    dph.reason = reason;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TX phase stopped: %s", dph_reason_text(reason));
    arrc_dphase_log_result();
    dph_push(MSG_ARRC_DPHASE_STATUS);
}

MAV_RESULT Copter::arrc_dphase_command(const mavlink_command_int_t &packet)
{
    const uint8_t action = uint8_t(packet.param1);
    if (action == ARRC_DPHASE_ACTION_STOP) {
        if (arrc_dphase_running()) {
            arrc_dphase_stop(ARRC_DPHASE_REASON_COMMAND);
        }
        return MAV_RESULT_ACCEPTED;
    }
    if (action != ARRC_DPHASE_ACTION_START) {
        return MAV_RESULT_DENIED;
    }

    auto deny = [this](uint8_t reason) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TX phase sweep: %s", dph_reason_text(reason));
        return (reason == ARRC_DPHASE_REASON_BUSY) ? MAV_RESULT_TEMPORARILY_REJECTED : MAV_RESULT_DENIED;
    };
    if (arrc_dphase_running()) {
        return deny(ARRC_DPHASE_REASON_BUSY);
    }

    // settings: 0 takes the default; params 5 and 6 arrive as the integer x and y of COMMAND_INT
    const float span = is_zero(packet.param2) ? DPH_MAX_SPAN_DEG : packet.param2;
    const float step = is_zero(packet.param3) ? 10.0f : packet.param3;
    const float settle = is_zero(packet.param4) ? 3.0f : packet.param4;
    const int32_t dwell = (packet.x == 0) ? 5 : packet.x;
    const uint8_t options = uint8_t(packet.y);
    if (!(span >= 5 && span <= DPH_MAX_SPAN_DEG) || !(step >= 1 && step <= 30) || !(settle >= 0.5f && settle <= 30) ||
        dwell < 1 || dwell > 60) {
        return deny(ARRC_DPHASE_REASON_SETTINGS);
    }
    // angles from -span to +span; the last one is +span even when 2*span is not a multiple of the step
    uint16_t n_one_way = uint16_t(floorf(2 * span / step + 1e-3f)) + 1;
    if (-span + (n_one_way - 1) * step < span - 1e-3f) {
        n_one_way++;
    }
    const uint16_t n_steps = (options & ARRC_DPHASE_OPTIONS_ONE_WAY) ? n_one_way : 2 * n_one_way - 1;
    if (n_one_way < 4 || n_steps > DPH_MAX_STEPS) {
        return deny(ARRC_DPHASE_REASON_SETTINGS);
    }
    // the probe is mounted one way or the other
    const bool probe_h = (options & ARRC_DPHASE_OPTIONS_PROBE_H) != 0;
    const bool probe_v = (options & ARRC_DPHASE_OPTIONS_PROBE_V) != 0;
    if (probe_h && probe_v) {
        return deny(ARRC_DPHASE_REASON_SETTINGS);
    }
    // the fit: the command's choice, else the parameter
    const bool fit_3 = (options & ARRC_DPHASE_OPTIONS_FIT_3TERM) != 0;
    const bool fit_4 = (options & ARRC_DPHASE_OPTIONS_FIT_4PSI) != 0;
    if (fit_3 && fit_4) {
        return deny(ARRC_DPHASE_REASON_SETTINGS);
    }

    if (arrc_align_running()) {
        return deny(ARRC_DPHASE_REASON_ALIGNMENT);
    }
    if (!arrc_dphase_vehicle_ok()) {
        return deny(ARRC_DPHASE_REASON_VEHICLE);
    }
    const uint8_t gmode = camera_mount.get_ARRC_gimbal_mode();
    if (gmode > ARRC_GIMBAL_MODE_VPOL) {
        return deny(ARRC_DPHASE_REASON_GIMBAL_MODE);
    }
    // pointing at a location (the ROI on the AUT) in any mode; the parallel-planes solution follows the switch
    uint8_t mode, flags;
    float pol_err;
    if (!camera_mount.get_ARRC_solution(mode, flags, pol_err) || (flags & ARRC_GIMBAL_FLAGS_TOO_CLOSE)) {
        return deny(ARRC_DPHASE_REASON_NO_SOLUTION);
    }

    // the RF source in use: the SDR while detected, otherwise the LB5900
    bool sdr = false;
#if AP_ARRC_SDR_ENABLED
    {
        mavlink_arrc_sdr_meas_t meas;
        uint32_t count;
        sdr = arrc_sdr_streaming() && ARRC_SDR.detected() && ARRC_SDR.meas_fresh(1000) &&
              ARRC_SDR.get_meas(meas, count) && (meas.flags & ARRC_MEAS_FLAGS_VALID);
    }
#endif
    if (!sdr) {
        uint32_t count;
        float power_dbm;
        if (!(arrc_lb5900_streaming() && arrc_align_sample(false, count, power_dbm))) {
            return deny(ARRC_DPHASE_REASON_RF_SOURCE);
        }
    }

    dph.run_id++;
    dph.sdr = sdr;
    dph.span = span;
    dph.step = step;
    dph.settle = settle;
    dph.dwell = uint8_t(dwell);
    dph.n_one_way = uint8_t(n_one_way);
    dph.n_steps = uint8_t(n_steps);
    dph.flags = (sdr ? ARRC_DPHASE_FLAGS_SDR : 0) | ((options & ARRC_DPHASE_OPTIONS_ONE_WAY) ? ARRC_DPHASE_FLAGS_ONE_WAY : 0) |
                (probe_h ? ARRC_DPHASE_FLAGS_PROBE_H : 0) | (probe_v ? ARRC_DPHASE_FLAGS_PROBE_V : 0);
    dph.probe_deg = probe_v ? 90.0f : 0.0f;
#if USER_PARAMS_ENABLED
    const bool param_4psi = g2.user_parameters.get_txdiff_fit() == 0;
#else
    const bool param_4psi = true;
#endif
    dph.fit_4psi = fit_4 || (!fit_3 && param_4psi);
    dph.term_d = dph.term_a = dph.term_b = dph.c4 = dph.e4 = dph.p_max_dbm = NaNf;
    dph.reason = ARRC_DPHASE_REASON_NONE;
    dph.idx = 0;
    dph.n_done = 0;
    dph.fit_r = NaNf;
    dph.result = {NaNf, NaNf, NaNf, NaNf, NaNf, false};
    dph.state = ARRC_DPHASE_STATE_MOVING;
    dph.state_ms = AP_HAL::millis();
    dph.run_start_ms = dph.state_ms;
    dph.start_gmode = gmode;
    if (gmode != ARRC_GIMBAL_MODE_PARALLEL_PLANES) {
        camera_mount.set_ARRC_gimbal_mode(ARRC_GIMBAL_MODE_PARALLEL_PLANES);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: MNT1_ARRC_GMODE %u -> 1 for the sweep", unsigned(gmode));
    }
    camera_mount.set_ARRC_parallel_rotation(dph_angle(0));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase sweep %u: +-%.0f by %.0f, %u angles (%s)", unsigned(dph.run_id),
                  span, step, unsigned(n_steps), sdr ? "SDR" : "LB5900");
    if (probe_h || probe_v) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: probe %s at angle 0", probe_v ? "V" : "H");
    }
    dph_push(MSG_ARRC_DPHASE_STATUS);
    return MAV_RESULT_ACCEPTED;
}

// 10 Hz from user_ARRC_gimbal()
void Copter::arrc_dphase_update()
{
    if (!arrc_dphase_running()) {
        return;
    }
    if (!arrc_dphase_vehicle_ok()) {
        arrc_dphase_stop(ARRC_DPHASE_REASON_VEHICLE);
        return;
    }
    // a mode set by someone else stops the sweep and is kept
    if (camera_mount.get_ARRC_gimbal_mode() != ARRC_GIMBAL_MODE_PARALLEL_PLANES) {
        arrc_dphase_stop(ARRC_DPHASE_REASON_GIMBAL_MODE);
        return;
    }
    uint8_t mode, flags;
    float pol_err;
    const bool have = camera_mount.get_ARRC_solution(mode, flags, pol_err);
    if (!have || mode != ARRC_GIMBAL_MODE_PARALLEL_PLANES || (flags & ARRC_GIMBAL_FLAGS_TOO_CLOSE)) {
        // right after the switch the latest solution may still be the previous mode's
        if (AP_HAL::millis() - dph.run_start_ms < DPH_MODE_GRACE_MS) {
            return;
        }
        arrc_dphase_stop(ARRC_DPHASE_REASON_NO_SOLUTION);
        return;
    }
    if (flags & (ARRC_GIMBAL_FLAGS_ROLL_LIMIT | ARRC_GIMBAL_FLAGS_PITCH_LIMIT)) {
        arrc_dphase_stop(ARRC_DPHASE_REASON_LIMIT);
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    switch (dph.state) {
    case ARRC_DPHASE_STATE_MOVING: {
        // settle, then wait until the command stops slewing and the measured angle is within the tolerance
        const uint32_t settle_ms = uint32_t(dph.settle * 1000) + ((dph.idx == 0) ? DPH_FIRST_EXTRA_MS : 0);
        const uint32_t waited_ms = now_ms - dph.state_ms;
        if (waited_ms < settle_ms) {
            return;
        }
        const bool slewing = (flags & 8U) != 0;     // ARRC_SLEW_LIMIT of the mount backend
        const bool off = isfinite(pol_err) && fabsf(pol_err) > DPH_ARRIVE_TOL_DEG;
        if (slewing || off) {
            if (waited_ms > settle_ms + DPH_ARRIVE_TIMEOUT_MS) {
                arrc_dphase_stop(ARRC_DPHASE_REASON_GIMBAL);
            }
            return;
        }
        // a reading already there may be from before the gimbal arrived: average only the next ones
        uint32_t count = 0;
        float power_dbm = 0, weight = 1;
        uint16_t meas_flags = 0;
        IGNORE_RETURN(arrc_dphase_sample(count, power_dbm, meas_flags, weight));
        dph.last_count = count;
        dph.sum_mw = dph.sum_w = dph.sum_db = dph.sum_db2 = dph.sum_pol = 0;
        dph.n = dph.n_rej = dph.n_pol = dph.meas_flags = 0;
#if AP_ARRC_SDR_ENABLED
        if (dph.sdr) {
            ARRC_SDR.accum_start(DPH_REJECT_FLAGS);
        }
#endif
        dph.state = ARRC_DPHASE_STATE_SAMPLING;
        dph.state_ms = now_ms;
        dph.last_sample_ms = now_ms;
        dph_push(MSG_ARRC_DPHASE_STATUS);
        break;
    }

    case ARRC_DPHASE_STATE_SAMPLING: {
        if (isfinite(pol_err)) {
            dph.sum_pol += pol_err;
            dph.n_pol++;
        }
#if AP_ARRC_SDR_ENABLED
        if (dph.sdr) {
            // every measurement since the averaging started (the companion sends faster than this loop runs)
            const auto &a = ARRC_SDR.accum();
            dph.sum_mw = a.sum_mw;
            dph.sum_w = a.sum_w;
            dph.sum_db = a.sum_db;
            dph.sum_db2 = a.sum_db2;
            dph.n = a.n;
            dph.n_rej = a.n_rej;
            dph.meas_flags = a.flags;
            if (a.last_ms != 0 && int32_t(a.last_ms - dph.last_sample_ms) > 0) {
                dph.last_sample_ms = a.last_ms;
            }
        }
#endif
        uint32_t count = 0;
        float power_dbm = 0, weight = 1;
        uint16_t meas_flags = 0;
        const bool valid = !dph.sdr && arrc_dphase_sample(count, power_dbm, meas_flags, weight);
        if (!dph.sdr && count != dph.last_count) {
            dph.last_count = count;
            if (valid) {
                dph.last_sample_ms = now_ms;
                if (meas_flags & DPH_REJECT_FLAGS) {
                    dph.n_rej++;
                } else {
                    dph.sum_mw += weight * powf(10.0f, 0.1f * power_dbm);
                    dph.sum_w += weight;
                    dph.sum_db += power_dbm;
                    dph.sum_db2 += double(power_dbm) * power_dbm;
                    dph.n++;
                    dph.meas_flags |= meas_flags;
                }
            }
        }
        if (now_ms - dph.last_sample_ms > DPH_SENSOR_TIMEOUT_MS) {
            arrc_dphase_stop(ARRC_DPHASE_REASON_RF_SOURCE);
            return;
        }
        if (now_ms - dph.state_ms < uint32_t(dph.dwell) * 1000) {
            return;
        }
        if (dph.n < DPH_MIN_SAMPLES) {
            arrc_dphase_stop(ARRC_DPHASE_REASON_FEW_SAMPLES);
            return;
        }

#if AP_ARRC_SDR_ENABLED
        ARRC_SDR.accum_stop();
#endif
        // record the angle
        auto &s = dph.steps[dph.idx];
        s.time_ms = now_ms;
        s.psi = dph_angle(dph.idx);
        s.psi_meas = (dph.n_pol > 0) ? float(s.psi + dph.sum_pol / dph.n_pol) : NaNf;
        s.power_dbm = 10.0f * log10f(float(dph.sum_mw / dph.sum_w));
        const double mean_db = dph.sum_db / dph.n;
        const double var_db = dph.sum_db2 / dph.n - mean_db * mean_db;
        s.power_sd = (dph.n > 1) ? sqrtf(float(MAX(var_db, 0.0) * dph.n / (dph.n - 1))) : NaNf;
        s.n = dph.n;
        s.n_rej = dph.n_rej;
        s.meas_flags = dph.meas_flags;
        if (dph.meas_flags & ARRC_MEAS_FLAGS_NEAR_CLIP) {
            dph.flags |= ARRC_DPHASE_FLAGS_NEAR_CLIP;
        }
        if (dph.meas_flags & ARRC_MEAS_FLAGS_PREDICTED) {
            dph.flags |= ARRC_DPHASE_FLAGS_PREDICTED;
        }
        if (!isfinite(s.psi_meas)) {
            dph.flags |= ARRC_DPHASE_FLAGS_COMMANDED_ANGLE;
        }
        dph.n_done = dph.idx + 1;
        arrc_dphase_log_step(dph.idx);
        dph_push(MSG_ARRC_DPHASE_STEP);

        // next angle, or the fit
        dph.idx++;
        if (dph.idx < dph.n_steps) {
            camera_mount.set_ARRC_parallel_rotation(dph_angle(dph.idx));
            dph.state = ARRC_DPHASE_STATE_MOVING;
            dph.state_ms = now_ms;
            dph_push(MSG_ARRC_DPHASE_STATUS);
            return;
        }
        arrc_dphase_finish();
        break;
    }

    default:
        break;
    }
}

void Copter::arrc_dphase_finish()
{
    // the measured angle about the boresight, else the commanded one, turned to the angle of the probe's polarization
    float psi[DPH_MAX_STEPS];
    float power_mw[DPH_MAX_STEPS];
    for (uint8_t i = 0; i < dph.n_steps; i++) {
        const auto &s = dph.steps[i];
        psi[i] = (isfinite(s.psi_meas) ? s.psi_meas : s.psi) + dph.probe_deg;
        power_mw[i] = powf(10.0f, 0.1f * s.power_dbm);
    }
    ARRC_PolFitTerms terms;
    ARRC_TxDiffPhase res;
    double c4 = 0;
    double e4 = 0;
    // the 4 psi terms need a wide sweep with enough angles to be told from the others; else three terms, also when
    // that fit gives no result
    const bool with_4psi = dph.fit_4psi && dph.span >= DPH_4PSI_MIN_SPAN_DEG && dph.n_one_way >= DPH_4PSI_MIN_ANGLES &&
                           ARRC_fit_polarization_terms_4psi(psi, power_mw, dph.n_steps, terms, c4, e4) &&
                           ARRC_tx_diff_phase(terms, DPH_H_SIGN, res);
    if (!with_4psi && (!ARRC_fit_polarization_terms(psi, power_mw, dph.n_steps, terms) ||
                       !ARRC_tx_diff_phase(terms, DPH_H_SIGN, res))) {
        arrc_dphase_stop(ARRC_DPHASE_REASON_FIT);
        return;
    }
    dph.term_d = float(terms.D);
    dph.term_a = float(terms.A);
    dph.term_b = float(terms.B);
    dph.c4 = with_4psi ? float(c4) : NaNf;
    dph.e4 = with_4psi ? float(e4) : NaNf;
    dph.p_max_dbm = float(10.0 * log10(terms.p_max));
    // the peak is reported as a gimbal angle
    res.peak_deg = wrap_180(res.peak_deg - dph.probe_deg);
    if (res.peak_deg > 90.0f) {
        res.peak_deg -= 180.0f;
    } else if (res.peak_deg <= -90.0f) {
        res.peak_deg += 180.0f;
    }
    camera_mount.set_ARRC_parallel_rotation(0);
    arrc_dphase_restore_gmode();
    dph.result = res;
    dph.fit_r = terms.r;
    dph.flags |= ARRC_DPHASE_FLAGS_RESULT | (res.clamped ? ARRC_DPHASE_FLAGS_COS_CLAMPED : 0) |
                 (with_4psi ? ARRC_DPHASE_FLAGS_FIT_4PSI : 0);
    dph.state = ARRC_DPHASE_STATE_DONE;
    if (dph.flags & (ARRC_DPHASE_FLAGS_PROBE_H | ARRC_DPHASE_FLAGS_PROBE_V)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: |b| %.1f +-%.1f deg (probe %s)", res.beta_deg, res.sigma_deg,
                      (dph.flags & ARRC_DPHASE_FLAGS_PROBE_V) ? "V" : "H");
    } else {
        // probe orientation not given: a horizontal probe is assumed; for a vertical one it is the other value
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: |b| %.1f +-%.1f deg (or %.1f)", res.beta_deg, res.sigma_deg,
                      180.0f - res.beta_deg);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: ZDRtx %.2f dB, swing %.1f dB, R %.3f", res.zdr_db, res.swing_db,
                  terms.r);
    if (with_4psi) {
        // the 4 psi terms as percent of the fitted peak of the three-term part
        const double pk = 0.01 * (terms.D + sqrt(terms.A * terms.A + terms.B * terms.B));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: 4psi fit, sin %+.1f cos %+.1f %% of peak", c4 / pk, e4 / pk);
    } else if (dph.fit_4psi) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: sweep too narrow for 4psi, 3-term fit");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TX phase: three-term fit");
    }
    arrc_dphase_log_result();
    dph_push(MSG_ARRC_DPHASE_STATUS);
}

void Copter::arrc_dphase_log_step(uint8_t index)
{
#if HAL_LOGGING_ENABLED
    const auto &s = dph.steps[index];
// @LoggerMessage: ARDP
// @Description: ARRC transmit differential phase sweep: one averaged probe angle
// @Field: TimeUS: Time since system startup
// @Field: Run: Run counter
// @Field: Stp: Angle index
// @Field: Psi: Commanded probe angle about the boresight from the AUT y axis
// @Field: PsiM: Measured probe angle about the boresight (NaN without a gimbal attitude report)
// @Field: P: Mean power of the valid readings (dBm, averaged in mW)
// @Field: PSd: Standard deviation of the valid readings (dB)
// @Field: N: Valid readings averaged
// @Field: NRj: Readings rejected for their flags
// @Field: Flg: ARRC_MEAS_FLAGS of the averaged readings, ORed
    logger.Write("ARDP", "TimeUS,Run,Stp,Psi,PsiM,P,PSd,N,NRj,Flg", "s--dd-----", "F---------", "QHBffffHHH",
                 AP_HAL::micros64(),
                 dph.run_id,
                 index,
                 s.psi,
                 s.psi_meas,
                 s.power_dbm,
                 s.power_sd,
                 s.n,
                 s.n_rej,
                 s.meas_flags);
#endif
}

void Copter::arrc_dphase_log_result()
{
#if HAL_LOGGING_ENABLED
// @LoggerMessage: ARDR
// @Description: ARRC transmit differential phase sweep: end of a run
// @Field: TimeUS: Time since system startup
// @Field: Run: Run counter
// @Field: St: ARRC_DPHASE_STATE (3 done, 4 stopped)
// @Field: Rsn: ARRC_DPHASE_REASON why it stopped
// @Field: N: Angles averaged
// @Field: GM: MNT1_ARRC_GMODE before the run (put back at its end)
// @Field: Bet: abs(beta), H to the left and V up seen from behind the radar (180 - Bet with the opposite polarity)
// @Field: BSd: Standard error of Bet
// @Field: Zdr: Transmit H to V power ratio at the probe (dB)
// @Field: Sw: Peak to null ratio of the fitted curve (dB)
// @Field: Pk: Probe angle of the fitted peak
// @Field: R: Correlation coefficient of the fit
// @Field: Flg: ARRC_DPHASE_FLAGS (128 / 256: the result is for a probe that is horizontal / vertical at angle 0; 512: fit with the 4 psi terms)
    logger.Write("ARDR", "TimeUS,Run,St,Rsn,N,GM,Bet,BSd,Zdr,Sw,Pk,R,Flg", "s-----dd--d--", "F------------",
                 "QHBBBBffffffH",
                 AP_HAL::micros64(),
                 dph.run_id,
                 dph.state,
                 dph.reason,
                 dph.n_done,
                 dph.start_gmode,
                 dph.result.beta_deg,
                 dph.result.sigma_deg,
                 dph.result.zdr_db,
                 dph.result.swing_db,
                 dph.result.peak_deg,
                 dph.fit_r,
                 dph.flags);
    if (dph.flags & ARRC_DPHASE_FLAGS_RESULT) {
// @LoggerMessage: ARDF
// @Description: ARRC transmit differential phase sweep: the fitted curve P = D + A sin(2x) + B cos(2x) + C4 sin(4x) + E4 cos(4x), x the gimbal angle plus 90 deg for a vertical probe
// @Field: TimeUS: Time since system startup
// @Field: Run: Run counter
// @Field: Pmx: Largest averaged power, the unit of the terms
// @Field: D: Constant term
// @Field: A: sin(2x) term
// @Field: B: cos(2x) term
// @Field: C4: sin(4x) term (NaN: three-term fit)
// @Field: E4: cos(4x) term (NaN: three-term fit)
        logger.Write("ARDF", "TimeUS,Run,Pmx,D,A,B,C4,E4", "s-------", "F-------", "QHffffff",
                     AP_HAL::micros64(),
                     dph.run_id,
                     dph.p_max_dbm,
                     dph.term_d,
                     dph.term_a,
                     dph.term_b,
                     dph.c4,
                     dph.e4);
    }
#endif
}

void Copter::arrc_dphase_send_status(mavlink_channel_t chan) const
{
    const bool running = arrc_dphase_running();
    // no run since boot: the result fields are NaN, as after a run without a result
    const bool ran = dph.run_id != 0;
    mavlink_msg_arrc_dphase_status_send(chan,
                                        AP_HAL::millis(),
                                        ran ? dph.result.beta_deg : NaNf,
                                        ran ? dph.result.sigma_deg : NaNf,
                                        ran ? dph.result.zdr_db : NaNf,
                                        ran ? dph.result.swing_db : NaNf,
                                        ran ? dph.result.peak_deg : NaNf,
                                        ran ? dph.fit_r : NaNf,
                                        running ? dph_angle(dph.idx) : NaNf,
                                        dph.span,
                                        dph.step,
                                        dph.settle,
                                        dph.dwell,
                                        dph.run_id,
                                        dph.flags,
                                        dph.state,
                                        dph.reason,
                                        running ? dph.idx : dph.n_done,
                                        dph.n_steps);
}

bool Copter::arrc_dphase_send_step(mavlink_channel_t chan, uint32_t index) const
{
    if (index == UINT32_MAX) {
        if (dph.n_done == 0) {
            return false;
        }
        index = dph.n_done - 1;
    }
    if (index >= dph.n_done || !HAVE_PAYLOAD_SPACE(chan, ARRC_DPHASE_STEP)) {
        return false;
    }
    const auto &s = dph.steps[index];
    mavlink_msg_arrc_dphase_step_send(chan,
                                      AP_HAL::millis(),
                                      s.time_ms,
                                      s.psi,
                                      s.psi_meas,
                                      s.power_dbm,
                                      s.power_sd,
                                      s.n,
                                      s.n_rej,
                                      s.meas_flags,
                                      dph.run_id,
                                      uint8_t(index),
                                      dph.n_steps);
    return true;
}

#endif  // HAL_MOUNT_ENABLED
