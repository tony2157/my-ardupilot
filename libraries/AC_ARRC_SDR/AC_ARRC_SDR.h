#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#define ARRC_RPI_SEARCH_MS 180000

class AC_ARRC_SDR {
public:
    AC_ARRC_SDR(void);
    ~AC_ARRC_SDR(void){}

    void init(){}

    // mavlink message handler
    float get_pwr_c(void) { return _pwr_c; }
    float get_pwr_x(void) { return _pwr_x; }
    float get_phi_c(void) { return _phi_c; }
    float get_phi_x(void) { return _phi_x; }
    uint64_t get_local_timestamp(void) { return _local_timestamp_us; }
    uint64_t get_unix_timestamp(void) { return _time_unix; }
    uint32_t get_boot_timestamp(void) { return _time_boot; }
    void handle_message(const mavlink_message_t &msg);

private:
    uint64_t _local_timestamp_us;   // time when the sample was collected on the UAS (since boot)
    uint32_t _time_boot;            // time since companion computer boot
    uint64_t _time_unix;            // Unix time tagged by the companion computer
    float _pwr_c;                   // Pulse/CW power CoPol (dBm)
    float _pwr_x;                   // Pulse/CW power CrossPol (dBm)
    float _phi_c;                   // Phase CoPol (deg)
    float _phi_x;                   // Phase CrossPol (deg)
};