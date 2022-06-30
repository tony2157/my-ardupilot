#include "AP_ARRC_RFE.h"

AP_ARRC_RFE::AP_ARRC_RFE() :
    _initialised(false),
    _timestamp_us(0),
    _freq(0),
    _power(0)
{
}

void AP_ARRC_RFE::handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg)
{
    mavlink_arrc_sensor_raw_t arrc_message;
    mavlink_msg_arrc_sensor_raw_decode(&msg, &arrc_message);
    _timestamp_us = AP_HAL::micros64();
    _freq = arrc_message.values[0];
    _power = arrc_message.values[1];
}

// search for onboard computer in GCS_MAVLink routing table
void AP_ARRC_RFE::find_RPi()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    // return if search time has has passed
    if (AP_HAL::millis() > ARRC_RPI_SEARCH_MS) {
        gcs().send_text(MAV_SEVERITY_INFO,"No RPi found");
        return;
    }

    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_ONBOARD_CONTROLLER, _sysid, _compid, _chan)) {
        gcs().send_text(MAV_SEVERITY_INFO,"Found RPi sysID %d compID %d chan %d",_sysid, _compid, _chan);
        _initialised = true;
    }
}