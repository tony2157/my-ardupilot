/*
 * AP_ARRC_SDR - ARRC RFSonde onboard SDR (companion computer) interface
 *
 * The companion computer (MAV_COMP_ID_ONBOARD_COMPUTER) runs the SDR and sends:
 *   - ARRC_SDR_MEAS targeted at the autopilot (nominal 30 Hz): logged as SDRM and kept for the
 *     ARRC_SENSOR_RAW downlink to the GCS
 *   - ARRC_CC_STATUS and ARRC_SDR_STATUS broadcast at 1 Hz: logged as SDRS
 *   - ARRC_SDR_CONFIG and ARRC_SDR_MEAS_CONFIG echoes (op=APPLIED): logged as SDRC and SDRQ
 * GCS to companion traffic is routed by GCS_MAVLink; this class only consumes what the autopilot needs.
 * It also sends the companion the sensor samples it records with its data (ARRC_NAV_*, AP_ARRC_SDR_Nav.cpp).
 */
#pragma once

#include "AP_ARRC_SDR_config.h"
#include "AP_ARRC_RateCap.h"

#if AP_ARRC_SDR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/ap_message.h>

class AP_ARRC_SDR {
public:
    AP_ARRC_SDR(void);

    CLASS_NO_COPY(AP_ARRC_SDR);

    // consume a message received on chan. The companion is taken from the channel its heartbeat arrives on;
    // its messages on any other channel are ignored until that heartbeat has been missing for the timeout
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

    // run at 10 Hz. timeout_ms is the companion link timeout
    void update(uint32_t timeout_ms);

    // true if the companion heartbeat and an SDR with the device present were seen within the timeout
    bool detected() const { return _detected; }

    // true if the companion heartbeat was seen within the timeout
    bool link_ok() const { return _link_ok; }

    // channel the companion's heartbeat arrives on, false before the first one
    bool companion_chan(mavlink_channel_t &chan) const { chan = _chan; return _have_chan; }

    // false if the companion reports an ARRC dialect other than the one the autopilot was built with
    bool dialect_ok() const { return !_dialect_mismatch; }

    // ask the companion to stop transmitting: sent on the companion's channel and repeated until it
    // acknowledges (or 10 times). reason is a string literal for the GCS text
    void request_tx_stop(const char *reason);

    // write the companion's SDR configuration in effect (SDRC, SDRQ) again, e.g. when a log starts
    void log_config(void);

    // true if a measurement newer than max_age_ms is available
    bool meas_fresh(uint32_t max_age_ms) const;

    // latest co-polar power in dBm and the number of measurements received since boot (changes with each one).
    // False if there is none or it is not a valid measurement (ARRC_MEAS_FLAGS_VALID)
    bool get_copol(float &power_dbm, uint32_t &count) const;

    // send the latest measurement as ARRC_SENSOR_RAW on a channel. Each measurement is sent once per channel
    void send_sensor_raw(mavlink_channel_t chan);


    // sample streams to the companion computer, at the rate the autopilot logs the same data
    enum class NavStream : uint8_t { IMU, MAG, BARO, GPS, ECEF, GIMBAL, COUNT };

    // the stream's message is due on chan. A channel with an interval for it (MAV_CMD_SET_MESSAGE_INTERVAL) gets
    // the samples from the vehicle tasks below, at the lower of the log rate and the requested rate; without one
    // (MAV_CMD_REQUEST_MESSAGE) the latest sample is sent once
    void nav_due(NavStream stream, mavlink_channel_t chan);

    // a sample, called where the vehicle logs IMU, MAG, BARO and ARGM: sent to the channels with the stream
    void nav_imu_sample(void);
    void nav_mag_sample(void);
    void nav_baro_sample(void);
    void nav_gimbal_sample(void);
    // GPS and ECEF, called faster than the fix rate: each new fix is sent once
    void nav_gps_sample(void);

    // companion data session id (0 if unknown)
    uint32_t session_id(void) const { return _session_id; }

private:
    void handle_heartbeat(mavlink_channel_t chan, const mavlink_message_t &msg);
    void handle_command_ack(const mavlink_message_t &msg);
    void log_sdr_config(const mavlink_arrc_sdr_config_t &p);
    void log_meas_config(const mavlink_arrc_sdr_meas_config_t &p);
    bool send_tx_stop(void);
    // CRC32 the companion and Ludifly compute over the ARRC messages of their dialect (rfsonde_cc protocol.py)
    static bool compute_dialect_hash(uint32_t &hash);
    void handle_meas(const mavlink_message_t &msg);
    void handle_cc_status(const mavlink_message_t &msg);
    void handle_sdr_status(const mavlink_message_t &msg);
    void handle_sdr_config(const mavlink_message_t &msg);
    void handle_meas_config(const mavlink_message_t &msg);

    bool from_companion(const mavlink_message_t &msg) const;

    // interval a channel requested for a message, false if it did not (no stream)
    static bool chan_interval(uint8_t chan, ap_message id, uint16_t &interval_ms);


    // true if a sample of the stream goes to chan now: the channel requested it with an interval, the interval
    // has passed since the last one and the channel keeps tx room for the heartbeat, ACKs and the GCS downlink
    bool nav_ready(NavStream stream, uint8_t chan, uint16_t payload_bytes);
    // send the latest sample of the stream on chan
    void nav_send(NavStream stream, mavlink_channel_t chan);
    void send_nav_gps(mavlink_channel_t chan);
    void send_nav_ecef(mavlink_channel_t chan);

    AP_ARRC_RateCap _nav_cap[uint8_t(NavStream::COUNT)][MAVLINK_COMM_NUM_BUFFERS];
    uint64_t _gps_fix_us;       // latest GPS fix sent to the channels with the stream
    uint64_t _ecef_fix_us;

    // ARRC_SENSOR_RAW: sent as each measurement arrives, at most at the interval the channel requested
    AP_ARRC_RateCap _rf_cap[MAVLINK_COMM_NUM_BUFFERS];

    // companion link
    uint32_t _last_hb_ms;
    uint32_t _last_sdr_status_ms;
    uint32_t _sdr_flags;
    uint32_t _timeout_ms;
    mavlink_channel_t _chan;            // channel of the companion's heartbeat
    bool _have_chan;
    bool _link_ok;
    bool _detected;
    bool _ever_detected;
    bool _link_lost_reported;
    bool _sdr_lost_reported;

    // dialect check
    uint32_t _dialect_hash;             // the autopilot's
    uint32_t _cc_dialect_hash;          // the companion's (ARRC_CC_STATUS)
    bool _dialect_hash_valid;
    bool _have_cc_dialect_hash;
    bool _dialect_mismatch;

    // TX stop until acknowledged
    const char *_tx_stop_reason;
    uint32_t _tx_stop_ms;
    uint8_t _tx_stop_tries;
    bool _tx_stop_pending;

    // configuration in effect, rewritten when a log starts; companion logs at most at these rates
    mavlink_arrc_sdr_config_t _sdr_config;
    mavlink_arrc_sdr_meas_config_t _meas_config;
    bool _have_sdr_config;
    bool _have_meas_config;
    uint32_t _sdrm_log_ms;
    uint32_t _sdcc_log_ms;
    uint32_t _sdrs_log_ms;
    uint32_t _sdrc_log_ms;
    uint32_t _sdrq_log_ms;

    // latest companion status, logged as SDCC
    uint32_t _session_id;
    uint32_t _rec_mb;

    // latest measurement
    mavlink_arrc_sdr_meas_t _meas;
    uint32_t _meas_ms;
    uint32_t _meas_count;
    bool _have_meas;
    uint32_t _last_sent_seq[MAVLINK_COMM_NUM_BUFFERS];
    bool _sent_any[MAVLINK_COMM_NUM_BUFFERS];
};

#endif  // AP_ARRC_SDR_ENABLED
