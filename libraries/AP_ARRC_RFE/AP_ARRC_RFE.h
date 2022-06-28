#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


class AP_ARRC_RFE {
public:
    AP_ARRC_RFE(void);
    ~AP_ARRC_RFE(void){}

    // mavlink message handler
    float get_power(void) { return _power; }
    float get_freq(void) { return _freq; }
    uint64_t get_timestamp(void) { return _timestamp_us; }
    void handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg);

private:
    uint64_t _timestamp_us; // time since boot when the sample was collected 
    float _power; //voltage read by the ADC
    float _freq; //frequency at which the power was measured
};