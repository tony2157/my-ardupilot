#include "AP_ARRC_RFE.h"

AP_ARRC_RFE::AP_ARRC_RFE() :
    _initialised(false),
    _timestamp_us(0),
    _freq(0),
    _pwr_c(0),
    _pwr_x(0),
    _aux_1(0),
    _aux_2(0),
    _aux_3(0),
    _dfreq(5800)
{
}

void AP_ARRC_RFE::init(uint16_t dfreq){
    _dfreq = dfreq;
}

void AP_ARRC_RFE::handle_message(const mavlink_message_t &msg)
{
    mavlink_arrc_sensor_raw_t arrc_message;
    mavlink_msg_arrc_sensor_raw_decode(&msg, &arrc_message);
    _timestamp_us = AP_HAL::micros64();
    _freq = arrc_message.freq;
    _pwr_c = arrc_message.pwr_c;
    _pwr_x = arrc_message.pwr_x;
    _aux_1 = arrc_message.aux_1;
    _aux_2 = arrc_message.aux_2;
    _aux_3 = arrc_message.aux_3;
}

// search for onboard computer in GCS_MAVLink routing table
void AP_ARRC_RFE::find_RPi()
{
    // return immediately if initialised
    if (_initialised) {
        // Send configuration to RPi
        mavlink_msg_arrc_sensor_raw_send(_chan, 0, _dfreq, 0, 0, 0, 0, 0, 0);
        return;
    }

    // return if search time has passed
    if (AP_HAL::millis() > ARRC_RPI_SEARCH_MS) {
        //gcs().send_text(MAV_SEVERITY_INFO,"No RPi found");
        return;
    }

    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_ONBOARD_CONTROLLER, _sysid, _compid, _chan)) {
        gcs().send_text(MAV_SEVERITY_INFO,"Found RPi sysID %d compID %d chan %d",_sysid, _compid, _chan);
        _initialised = true;
    }
}