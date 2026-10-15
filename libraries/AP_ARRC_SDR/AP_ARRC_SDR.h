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

#if AP_ARRC_SDR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

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

    // sample streams to the companion computer, at the rate the autopilot logs the same data
    enum class NavStream : uint8_t { IMU, MAG, BARO, GPS, ECEF, GIMBAL, COUNT };

    // the stream's message is due on chan (the companion requested it with MAV_CMD_SET_MESSAGE_INTERVAL):
    // chan gets the stream. GPS and ECEF are sent from here, once per new fix, as they are logged
    void nav_due(NavStream stream, mavlink_channel_t chan);

    // a sample, called where the vehicle logs IMU, MAG, BARO and ARGM: sent to the channels with the stream
    void nav_imu_sample(void);
    void nav_mag_sample(void);
    void nav_baro_sample(void);
    void nav_gimbal_sample(void);

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

    // true if chan gets the stream (its message was due there in the last 2 s)
    bool nav_subscribed(NavStream stream, uint8_t chan) const;
    void send_nav_gps(mavlink_channel_t chan);
    void send_nav_ecef(mavlink_channel_t chan);

    uint32_t _nav_due_ms[uint8_t(NavStream::COUNT)][MAVLINK_COMM_NUM_BUFFERS];
    uint64_t _gps_sent_us[MAVLINK_COMM_NUM_BUFFERS];    // fix last sent on each channel
    uint64_t _ecef_sent_us[MAVLINK_COMM_NUM_BUFFERS];

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
