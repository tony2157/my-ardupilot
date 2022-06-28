#include "AP_ARRC_RFE.h"

AP_ARRC_RFE::AP_ARRC_RFE() :
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