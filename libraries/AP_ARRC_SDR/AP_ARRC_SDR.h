/*
 * AP_ARRC_SDR - ARRC RFSonde onboard SDR (companion computer) interface
 *
 * The companion computer (MAV_COMP_ID_ONBOARD_COMPUTER) runs the SDR and sends:
 *   - ARRC_SDR_MEAS targeted at the autopilot (nominal 30 Hz): logged as SDRM and kept for the
 *     ARRC_SENSOR_RAW downlink to the GCS
 *   - ARRC_CC_STATUS and ARRC_SDR_STATUS broadcast at 1 Hz: logged as SDRS
 *   - ARRC_SDR_CONFIG and ARRC_SDR_MEAS_CONFIG echoes (op=APPLIED): logged as SDRC and SDRQ
 * GCS to companion traffic is routed by GCS_MAVLink; this class only consumes what the autopilot needs.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#ifndef AP_ARRC_SDR_ENABLED
#define AP_ARRC_SDR_ENABLED 1
#endif

#if AP_ARRC_SDR_ENABLED

class AP_ARRC_SDR {
public:
    AP_ARRC_SDR(void);

    CLASS_NO_COPY(AP_ARRC_SDR);

    // consume a message received on any channel
    void handle_message(const mavlink_message_t &msg);

    // run at 10 Hz. timeout_ms is the companion link timeout
    void update(uint32_t timeout_ms);

    // true if the companion heartbeat and an SDR with the device present were seen within the timeout
    bool detected() const { return _detected; }

    // true if a measurement newer than max_age_ms is available
    bool meas_fresh(uint32_t max_age_ms) const;

    // send the latest measurement as ARRC_SENSOR_RAW on a channel. Each measurement is sent once per channel
    void send_sensor_raw(mavlink_channel_t chan);

    // ask the companion to stop transmitting. Returns false if no route to the companion is known
    bool send_tx_stop(void);

    // companion data session id (0 if unknown)
    uint32_t session_id(void) const { return _session_id; }

private:
    void handle_heartbeat(const mavlink_message_t &msg);
    void handle_meas(const mavlink_message_t &msg);
    void handle_cc_status(const mavlink_message_t &msg);
    void handle_sdr_status(const mavlink_message_t &msg);
    void handle_sdr_config(const mavlink_message_t &msg);
    void handle_meas_config(const mavlink_message_t &msg);

    bool from_companion(const mavlink_message_t &msg) const;

    // companion link
    uint32_t _last_hb_ms;
    uint32_t _last_sdr_status_ms;
    uint32_t _sdr_flags;
    bool _detected;
    bool _ever_detected;
    bool _link_lost_reported;

    // latest companion status, logged as SDCC
    uint32_t _session_id;
    uint32_t _rec_mb;

    // latest measurement
    mavlink_arrc_sdr_meas_t _meas;
    uint32_t _meas_ms;
    bool _have_meas;
    uint32_t _last_sent_seq[MAVLINK_COMM_NUM_BUFFERS];
    bool _sent_any[MAVLINK_COMM_NUM_BUFFERS];
};

#endif  // AP_ARRC_SDR_ENABLED
