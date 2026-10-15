#include "AP_ARRC_SDR.h"

#if AP_ARRC_SDR_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

// a measurement older than this is not sent to the GCS
static constexpr uint32_t MEAS_DOWNLINK_MAX_AGE_MS = 1000;

AP_ARRC_SDR::AP_ARRC_SDR(void) :
    _last_hb_ms(0),
    _last_sdr_status_ms(0),
    _sdr_flags(0),
    _detected(false),
    _ever_detected(false),
    _link_lost_reported(false),
    _session_id(0),
    _rec_mb(0),
    _meas_ms(0),
    _have_meas(false)
{
    memset(&_meas, 0, sizeof(_meas));
    memset(_last_sent_seq, 0, sizeof(_last_sent_seq));
    memset(_sent_any, 0, sizeof(_sent_any));
    memset(_nav_due_ms, 0, sizeof(_nav_due_ms));
    memset(_gps_sent_us, 0, sizeof(_gps_sent_us));
    memset(_ecef_sent_us, 0, sizeof(_ecef_sent_us));
}

bool AP_ARRC_SDR::from_companion(const mavlink_message_t &msg) const
{
    return msg.sysid == mavlink_system.sysid && msg.compid == MAV_COMP_ID_ONBOARD_COMPUTER;
}

void AP_ARRC_SDR::handle_message(const mavlink_message_t &msg)
{
    if (!from_companion(msg)) {
        return;
    }
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_heartbeat(msg);
        break;
    case MAVLINK_MSG_ID_ARRC_SDR_MEAS:
        handle_meas(msg);
        break;
    case MAVLINK_MSG_ID_ARRC_CC_STATUS:
        handle_cc_status(msg);
        break;
    case MAVLINK_MSG_ID_ARRC_SDR_STATUS:
        handle_sdr_status(msg);
        break;
    case MAVLINK_MSG_ID_ARRC_SDR_CONFIG:
        handle_sdr_config(msg);
        break;
    case MAVLINK_MSG_ID_ARRC_SDR_MEAS_CONFIG:
        handle_meas_config(msg);
        break;
    default:
        break;
    }
}

void AP_ARRC_SDR::handle_heartbeat(const mavlink_message_t &msg)
{
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    if (hb.type == MAV_TYPE_ONBOARD_CONTROLLER) {
        _last_hb_ms = AP_HAL::millis();
    }
}

void AP_ARRC_SDR::handle_meas(const mavlink_message_t &msg)
{
    mavlink_msg_arrc_sdr_meas_decode(&msg, &_meas);
    _meas_ms = AP_HAL::millis();
    _have_meas = true;

#if HAL_LOGGING_ENABLED
// @LoggerMessage: SDRM
// @Description: ARRC onboard SDR measurement (from the companion computer)
// @Field: TimeUS: Time since system startup when the measurement was received
// @Field: SampUS: Centre of the averaging window on the autopilot clock
// @Field: Seq: Sequence number
// @Field: PC: Co-pol power
// @Field: PX: Cross-pol power (NaN if unavailable)
// @Field: PhC: Co-pol phase (reference in PR)
// @Field: PhX: Cross-pol phase relative to co-pol
// @Field: SNR: Co-pol SNR in dB
// @Field: Nse: Co-pol noise power
// @Field: CFO: Carrier frequency offset
// @Field: TOA: Mean time of arrival within the PRI
// @Field: NA: Pulses or samples averaged
// @Field: NP: Predicted (undetected) pulses included
// @Field: Flg: Measurement flags (ARRC_MEAS_FLAGS)
// @Field: PR: Phase reference (ARRC_PHASE_REF)
// @Field: Sc: Producing script (255 = standard engine)
    AP::logger().WriteStreaming("SDRM",
                                "TimeUS,SampUS,Seq,PC,PX,PhC,PhX,SNR,Nse,CFO,TOA,NA,NP,Flg,PR,Sc",
                                "ss-RRdd-Rzs-----",
                                "FF-0000-000-----",
                                "QQIffffffffHHHBB",
                                AP_HAL::micros64(),
                                _meas.time_usec,
                                _meas.seq,
                                _meas.pwr_c,
                                _meas.pwr_x,
                                _meas.phi_c,
                                _meas.phi_x,
                                _meas.snr_db,
                                _meas.noise_dbm,
                                _meas.cfo_hz,
                                _meas.toa_s,
                                _meas.n_avg,
                                _meas.n_pred,
                                _meas.flags,
                                _meas.phase_ref,
                                _meas.script_id);
#endif
}

void AP_ARRC_SDR::handle_cc_status(const mavlink_message_t &msg)
{
    mavlink_arrc_cc_status_t p;
    mavlink_msg_arrc_cc_status_decode(&msg, &p);
    _session_id = p.session_id;

#if HAL_LOGGING_ENABLED
// @LoggerMessage: SDCC
// @Description: ARRC companion computer status
// @Field: TimeUS: Time since system startup
// @Field: Sess: Data session id on the companion computer
// @Field: Flg: Companion flags (ARRC_CC_FLAGS)
// @Field: St: Companion state (ARRC_CC_STATE)
// @Field: CPU: CPU load
// @Field: Tmp: CPU temperature
// @Field: Mem: Memory used in MiB
// @Field: Disk: Free data disk in MiB
// @Field: RTT: Time sync round trip
// @Field: RecMB: Data recorded in this session in MiB
    AP::logger().Write("SDCC",
                       "TimeUS,Sess,Flg,St,CPU,Tmp,Mem,Disk,RTT,RecMB",
                       "s---%O--s-",
                       "F---00--F-",
                       "QIHBBbHIII",
                       AP_HAL::micros64(),
                       p.session_id,
                       p.flags,
                       p.state,
                       p.cpu_load,
                       p.cpu_temp,
                       p.mem_used_mb,
                       p.disk_free_mb,
                       p.timesync_rtt_us,
                       _rec_mb);
#endif
}

void AP_ARRC_SDR::handle_sdr_status(const mavlink_message_t &msg)
{
    mavlink_arrc_sdr_status_t p;
    mavlink_msg_arrc_sdr_status_decode(&msg, &p);
    _last_sdr_status_ms = AP_HAL::millis();
    _sdr_flags = p.flags;
    _rec_mb = p.rec_mb;

#if HAL_LOGGING_ENABLED
// @LoggerMessage: SDRS
// @Description: ARRC onboard SDR status
// @Field: TimeUS: Time since system startup
// @Field: St: Engine state (ARRC_SDR_STATE)
// @Field: Md: Mode (ARRC_SDR_MODE)
// @Field: Flg: SDR flags (ARRC_SDR_FLAGS)
// @Field: Freq: Centre frequency
// @Field: SR: Sample rate
// @Field: BW: Analog bandwidth
// @Field: Rate: Measurement feed rate
// @Field: PRF: Configured or tracked PRF
// @Field: G0: RX1 gain in dB
// @Field: G1: RX2 gain in dB
// @Field: TG: TX gain in dB
// @Field: RT: RFIC temperature
// @Field: Ovr: RX overruns since start
// @Field: Drp: Recorder records dropped since start
// @Field: Sc: Running script (255 = none)
    AP::logger().Write("SDRS",
                       "TimeUS,St,Md,Flg,Freq,SR,BW,Rate,PRF,G0,G1,TG,RT,Ovr,Drp,Sc",
                       "s---zzzzz---O---",
                       "F---00000---B---",
                       "QBBIQIIfffffhIIB",
                       AP_HAL::micros64(),
                       p.state,
                       p.mode,
                       p.flags,
                       p.freq_hz,
                       p.sample_rate,
                       p.bandwidth,
                       p.meas_rate_hz,
                       p.prf_hz,
                       p.rx_gain_db[0],
                       p.rx_gain_db[1],
                       p.tx_gain_db,
                       p.rfic_temp,
                       p.overruns,
                       p.rec_drops,
                       p.active_script);
#endif
}

void AP_ARRC_SDR::handle_sdr_config(const mavlink_message_t &msg)
{
    mavlink_arrc_sdr_config_t p;
    mavlink_msg_arrc_sdr_config_decode(&msg, &p);
    if (p.op != ARRC_CONFIG_OP_APPLIED) {
        return;
    }

#if HAL_LOGGING_ENABLED
    char waveform[17] {};
    memcpy(waveform, p.waveform, sizeof(p.waveform));
// @LoggerMessage: SDRC
// @Description: ARRC onboard SDR main configuration in effect (TX shares frequency, waveform and PRF)
// @Field: TimeUS: Time since system startup
// @Field: Req: Request id that caused the change (0 = report)
// @Field: Res: Result (ARRC_CONFIG_RESULT)
// @Field: Mode: Mode (ARRC_SDR_MODE)
// @Field: Freq: Centre frequency
// @Field: SR: Sample rate
// @Field: BW: Analog bandwidth
// @Field: PRF: PRF
// @Field: Pol: Radar transmit polarisation (ARRC_RADAR_TX_POL)
// @Field: G0: RX1 gain in dB
// @Field: G1: RX2 gain in dB
// @Field: TG: TX1 gain in dB
// @Field: Prof: Port profile index
// @Field: Rol: RX1 role + 16 * RX2 role (ARRC_PORT_ROLE)
// @Field: GM: Gain mode (ARRC_GAIN_MODE)
// @Field: WF: Waveform name
    AP::logger().Write("SDRC",
                       "TimeUS,Req,Res,Mode,Freq,SR,BW,PRF,Pol,G0,G1,TG,Prof,Rol,GM,WF",
                       "s---zzzz--------",
                       "F---0000--------",
                       "QHBBQIIfBfffBBBN",
                       AP_HAL::micros64(),
                       p.req_id,
                       p.result,
                       p.mode,
                       p.freq_hz,
                       p.sample_rate,
                       p.bandwidth,
                       p.prf_hz,
                       p.radar_tx_pol,
                       p.rx_gain_db[0],
                       p.rx_gain_db[1],
                       p.tx_gain_db[0],
                       p.port_profile,
                       uint8_t((p.rx_role[0] & 0x0F) | ((p.rx_role[1] & 0x0F) << 4)),
                       p.gain_mode,
                       waveform);
#endif
}

void AP_ARRC_SDR::handle_meas_config(const mavlink_message_t &msg)
{
    mavlink_arrc_sdr_meas_config_t p;
    mavlink_msg_arrc_sdr_meas_config_decode(&msg, &p);
    if (p.op != ARRC_CONFIG_OP_APPLIED) {
        return;
    }

#if HAL_LOGGING_ENABLED
    char template_name[17] {};
    memcpy(template_name, p.template_name, sizeof(p.template_name));
// @LoggerMessage: SDRQ
// @Description: ARRC onboard SDR measurement and recording configuration in effect
// @Field: TimeUS: Time since system startup
// @Field: Req: Request id that caused the change (0 = report)
// @Field: Res: Result (ARRC_CONFIG_RESULT)
// @Field: Sg: Signal type (ARRC_SIGNAL_TYPE)
// @Field: Dt: Detector (ARRC_DETECTOR)
// @Field: AM: Averaging mode (ARRC_AVG_MODE)
// @Field: AN: Pulses or samples per average
// @Field: Rate: Maximum feed rate
// @Field: TM: Threshold mode (ARRC_THRESHOLD_MODE)
// @Field: Thr: Threshold
// @Field: RM: Recording mode (ARRC_REC_MODE)
// @Field: GM: Gate mode (ARRC_GATE_MODE)
// @Field: Pre: Recording window before the pulse
// @Field: Post: Recording window after the pulse
// @Field: NC: Noise correction enabled
// @Field: Tpl: Matched-filter template (empty = main waveform)
    AP::logger().Write("SDRQ",
                       "TimeUS,Req,Res,Sg,Dt,AM,AN,Rate,TM,Thr,RM,GM,Pre,Post,NC,Tpl",
                       "s------z----ss--",
                       "F------0----00--",
                       "QHBBBBIfBfBBffBN",
                       AP_HAL::micros64(),
                       p.req_id,
                       p.result,
                       p.signal,
                       p.detector,
                       p.avg_mode,
                       p.avg_n,
                       p.output_rate_hz,
                       p.threshold_mode,
                       p.threshold,
                       p.rec_mode,
                       p.gate_mode,
                       p.gate_pre_s,
                       p.gate_post_s,
                       p.noise_correct,
                       template_name);
#endif
}

void AP_ARRC_SDR::update(uint32_t timeout_ms)
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool link_ok = _last_hb_ms != 0 && now_ms - _last_hb_ms < timeout_ms;
    const bool sdr_ok = _last_sdr_status_ms != 0 && now_ms - _last_sdr_status_ms < timeout_ms &&
                        (_sdr_flags & ARRC_SDR_FLAGS_DEVICE_PRESENT) != 0;

    const bool detected = link_ok && sdr_ok;
    if (detected && !_detected) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SDR: companion and SDR detected");
    }
    _detected = detected;
    _ever_detected |= detected;

    // report a lost companion once, and its return
    if (_ever_detected && !link_ok && !_link_lost_reported) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SDR: companion link lost");
        _link_lost_reported = true;
    } else if (link_ok && _link_lost_reported) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SDR: companion link restored");
        _link_lost_reported = false;
    }
}

bool AP_ARRC_SDR::meas_fresh(uint32_t max_age_ms) const
{
    return _have_meas && AP_HAL::millis() - _meas_ms < max_age_ms;
}

void AP_ARRC_SDR::send_sensor_raw(mavlink_channel_t chan)
{
    if (!meas_fresh(MEAS_DOWNLINK_MAX_AGE_MS) || chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return;
    }
    // send each measurement once per channel
    if (_sent_any[chan] && _last_sent_seq[chan] == _meas.seq) {
        return;
    }
    mavlink_msg_arrc_sensor_raw_send(chan,
                                     AP_HAL::millis(),
                                     _meas.time_usec,
                                     _meas.pwr_c,
                                     _meas.pwr_x,
                                     _meas.phi_c,
                                     _meas.phi_x,
                                     _meas.snr_db,
                                     _meas.flags,
                                     _meas.n_avg);
    _last_sent_seq[chan] = _meas.seq;
    _sent_any[chan] = true;
}

bool AP_ARRC_SDR::send_tx_stop(void)
{
    uint8_t sysid;
    mavlink_channel_t chan;
    if (!GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_ONBOARD_CONTROLLER, MAV_COMP_ID_ONBOARD_COMPUTER, sysid, chan)) {
        return false;
    }
    if (!HAVE_PAYLOAD_SPACE(chan, COMMAND_LONG)) {
        return false;
    }
    mavlink_msg_command_long_send(chan,
                                  sysid,
                                  MAV_COMP_ID_ONBOARD_COMPUTER,
                                  MAV_CMD_ARRC_SDR_CONTROL,
                                  0,
                                  ARRC_SDR_ACTION_STOP,
                                  ARRC_SDR_SUBSYS_TX,
                                  0, 0, 0, 0, 0);
    return true;
}

#endif  // AP_ARRC_SDR_ENABLED
