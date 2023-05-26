#include "AC_ARRC_LB680A.h"

AC_ARRC_LB680A::AC_ARRC_LB680A() :
    _initialised(false),
    _timestamp_us(0),
    _pwr(0),
    _pkpwr(0),
    _avgpwr(0),
    _dcyc(0),
    _dfreq(3070),
    _davg(20),
    _dmode(1)
{
}

void AC_ARRC_LB680A::init(uint16_t dfreq, uint16_t davg, uint16_t dmode){
    _dfreq = dfreq;
    _davg = davg;
    _dmode = dmode;
}

void AC_ARRC_LB680A::handle_message(const mavlink_message_t &msg)
{
    mavlink_arrc_sensor_raw_t arrc_message;
    mavlink_msg_arrc_sensor_raw_decode(&msg, &arrc_message);
    _timestamp_us = AP_HAL::micros64();
    _pwr = arrc_message.pwr;
    _pkpwr = arrc_message.pkpwr;
    _avgpwr = arrc_message.avgpwr;
    _dcyc = arrc_message.dcyc;
}

// search for onboard computer in GCS_MAVLink routing table
void AC_ARRC_LB680A::find_RPi()
{
    // return immediately if initialised
    if (_initialised) {
        // Send configuration to RPi
        mavlink_msg_arrc_sensor_raw_send(_chan, AP_HAL::millis(), _dfreq, _davg, _dmode, 0, 0, 0, 0);
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