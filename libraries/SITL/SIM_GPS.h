/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulate GPS sensors

  Usage example:
param set SERIAL5_PROTOCOL 5

     sim_vehicle.py -D --console --map -A --serial5=sim:gps:2
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_GPS_ENABLED

#include <sys/time.h>
#include "SIM_SerialDevice.h"
#include <AP_HAL/AP_HAL.h>
#include <random>

namespace SITL {

// for delay simulation:
struct GPS_Data {
    uint32_t timestamp_ms;
    double latitude;
    double longitude;
    float altitude;
    double speedN;
    double speedE;
    double speedD;
    double yaw_deg;
    double roll_deg;
    double pitch_deg;
    bool have_lock;
    float horizontal_acc;
    float vertical_acc;
    float speed_acc;
    uint8_t num_sats;

    // Get course over ground [rad], where 0 = North in WGS-84 coordinate system.
    // Calculated from 2D velocity.
    float ground_track_rad() const WARN_IF_UNUSED;

    // Get 2D speed [m/s] in WGS-84 coordinate system
    float speed_2d() const WARN_IF_UNUSED;
};

// RFSonde GNSS error model (SIM_GPS_HBIAS ... SIM_GPS_SEED).
// Per axis (north, east, down) the position error is a first-order Gauss-Markov process (slowly
// varying error from atmosphere, orbits, multipath) plus white noise, and the velocity gets white
// noise. The Gauss-Markov state is kept in units of its standard deviation and propagated with the
// exact discretisation x <- phi*x + sqrt(1 - phi^2)*n, phi = exp(-dt/tau), so the statistics do not
// depend on the GPS update rate and a change of the configured sizes applies immediately.
class GNSSErrorModel {
public:
    struct Params {
        float h_bias;       // horizontal correlated error, 1-sigma per axis (m)
        float v_bias;       // vertical correlated error, 1-sigma (m)
        float tau;          // correlation time of the correlated error (s)
        float h_white;      // horizontal white noise, 1-sigma per axis (m)
        float v_white;      // vertical white noise, 1-sigma (m)
        float vel_noise;    // velocity white noise, 1-sigma per axis (m/s)
        uint32_t seed;      // 0 = random seed
    };
    struct Error {
        double north, east, down;   // position error (m)
        float vel_north, vel_east, vel_down;   // velocity error (m/s)
    };

    static bool enabled(const Params &p) {
        return p.h_bias > 0 || p.v_bias > 0 || p.h_white > 0 || p.v_white > 0 || p.vel_noise > 0;
    }

    // advance the model to now_ms and return the current error
    Error update(const Params &p, uint32_t now_ms, uint8_t instance);

private:
    std::mt19937 generator;
    std::normal_distribution<double> unit_normal{0.0, 1.0};
    double state[3] {};         // correlated error per axis in units of its sigma
    uint32_t last_ms;
    uint32_t seed_used;
    bool initialised;
};

class GPS_Backend {
public:
    CLASS_NO_COPY(GPS_Backend);

    GPS_Backend(class GPS &front, uint8_t _instance);
    virtual ~GPS_Backend() {}

    // 0 baud means "unset" i.e. baud-rate checks should not apply
    virtual uint32_t device_baud() const { return 0; }

    ssize_t write_to_autopilot(const char *p, size_t size) const;
    ssize_t read_from_autopilot(char *buffer, size_t size) const;

    // read and process config from autopilot (e.g.)
    virtual void update_read();
    // writing fix information to autopilot (e.g.)
    virtual void publish(const GPS_Data *d) = 0;

    struct GPS_TOW {
        // Number of weeks since midnight 5-6 January 1980
        uint16_t week;
        // Time since start of the GPS week [mS]
        uint32_t ms;
    };

    static GPS_TOW gps_time();

protected:

    uint8_t instance;
    GPS &front;

    class SIM *_sitl;

    static void simulation_timeval(struct timeval *tv);
};

class GPS : public SerialDevice {
public:

    CLASS_NO_COPY(GPS);

    enum Type {
        NONE  =  0,
#if AP_SIM_GPS_UBLOX_ENABLED
        UBLOX =  1,
#endif
#if AP_SIM_GPS_NMEA_ENABLED
        NMEA  =  5,
#endif
#if AP_SIM_GPS_SBP_ENABLED
        SBP   =  6,
#endif
#if AP_SIM_GPS_FILE_ENABLED
        FILE  =  7,
#endif
#if AP_SIM_GPS_NOVA_ENABLED
        NOVA  =  8,
#endif
#if AP_SIM_GPS_SBP2_ENABLED
        SBP2  =  9,
#endif
#if AP_SIM_GPS_SBF_ENABLED
        SBF = 10, //matches GPS_TYPE 
#endif
#if AP_SIM_GPS_TRIMBLE_ENABLED
        TRIMBLE  = 11, // matches GPS1_TYPE
#endif
#if AP_SIM_GPS_MSP_ENABLED
        MSP   = 19,
#endif
    };

    GPS(uint8_t _instance);

    // update state
    void update();

    ssize_t write_to_autopilot(const char *p, size_t size) const override;

    uint32_t device_baud() const override;  // 0 meaning unset

private:

    uint8_t instance;

    // The last time GPS data was written [mS]
    uint32_t last_write_update_ms;

    // last 20 samples, allowing for up to 20 samples of delay
    GPS_Data _gps_history[20];

    // state of jamming simulation
    struct {
        uint32_t last_jam_ms;
        uint32_t jam_start_ms;
        uint32_t last_sats_change_ms;
        uint32_t last_vz_change_ms;
        uint32_t last_vel_change_ms;
        uint32_t last_pos_change_ms;
        uint32_t last_acc_change_ms;
        double latitude;
        double longitude;
    } jamming[2];

    bool _gps_has_basestation_position;
    GPS_Data _gps_basestation_data;

    void simulate_jamming(GPS_Data &d);

    // get delayed data
    GPS_Data interpolate_data(const GPS_Data &d, uint32_t delay_ms);

    uint8_t allocated_type;
    GPS_Backend *backend;
    void check_backend_allocation();

    // RFSonde GNSS error model
    GNSSErrorModel error_model;
};

}

#endif  // AP_SIM_GPS_ENABLED
