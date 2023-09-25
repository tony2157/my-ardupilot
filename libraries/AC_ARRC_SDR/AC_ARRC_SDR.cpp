#include "AC_ARRC_SDR.h"

AC_ARRC_SDR::AC_ARRC_SDR() :
    _local_timestamp_us(0),
    _time_boot(0),
    _time_unix(0),
    _pwr_c(0),
    _pwr_x(0),
    _phi_c(0),
    _phi_x(0)
{
}

void AC_ARRC_SDR::handle_message(const mavlink_message_t &msg)
{
    mavlink_arrc_sensor_raw_t arrc_message;
    mavlink_msg_arrc_sensor_raw_decode(&msg, &arrc_message);
    _local_timestamp_us = AP_HAL::micros64();
    _time_boot = arrc_message.time_boot_ms;
    _time_unix = arrc_message.time_usec;
    _pwr_c = arrc_message.pwr_c;
    _pwr_x = arrc_message.pwr_x;
    _phi_c = arrc_message.phi_c;
    _phi_x = arrc_message.phi_x;
}