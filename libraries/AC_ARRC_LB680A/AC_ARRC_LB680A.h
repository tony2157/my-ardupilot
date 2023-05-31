#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define ARRC_RPI_SEARCH_MS 180000

class AC_ARRC_LB680A {
public:
    AC_ARRC_LB680A(void);
    ~AC_ARRC_LB680A(void){}

    void init(uint16_t dfreq, uint16_t davg, uint16_t dmode);

    // search for onboard computer in GCS_MAVLink routing table
    void find_RPi();

    // mavlink message handler
    float get_pwr(void) { return _pwr; }
    float get_pkpwr(void) { return _pkpwr; }
    float get_avgpwr(void) { return _avgpwr; }
    float get_dcyc(void) { return _dcyc; }
    uint64_t get_timestamp(void) { return _timestamp_us; }
    void handle_message(const mavlink_message_t &msg);

private:
    bool _initialised;
    uint8_t _sysid;                 // sysid of onboard computer
    uint8_t _compid;                // component id of onboard computer
    mavlink_channel_t _chan;        // mavlink channel used to communicate with onboard computer
    uint64_t _timestamp_us;         // time when the sample was collected (since boot)
    float _pwr;                   // Pulse/CW power (dBm)
    float _pkpwr;                   // Peak power (dBm)
    float _avgpwr;                   // Average power (dBm)
    float _dcyc;                   // Duty cycle
    uint16_t _dfreq;                 // Desire frequency to sample
    uint16_t _davg;                 // Desire average length
    uint16_t _dmode;                 // Desire Pulse or CW mode
};