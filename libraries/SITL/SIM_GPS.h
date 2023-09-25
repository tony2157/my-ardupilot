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

#if HAL_SIM_GPS_ENABLED

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

// Stochastic model for GNSS bias and noise
class GNSSStochasticModel {
    std::mt19937 generator; // Mersenne Twister random engine
    std::normal_distribution<double> gnss_velocity_distribution;

public:
    GNSSStochasticModel(double vel_std, double noise_std)
        : gnss_velocity_distribution(0.0, vel_std),
          gnss_vel_std(vel_std), lambda(1.0), last_now(AP_HAL::millis()),
          lat_bias(0.0), lng_bias(0.0), vertical_bias(0.0){
        // Seed random engine with a high-quality random seed
        std::random_device rd;
        generator.seed(rd());
    }

    void set_gnss_velocity_std(double vel_std, double l) {
        gnss_vel_std = vel_std;
        lambda = l;
        lat_bias = 0.0;
        lng_bias = 0.0;
        vertical_bias = 0.0;
    }

    void updateBias() {
        // Update bias using a random walk
        gnss_velocity_distribution = std::normal_distribution<double>(-lat_bias*lambda, gnss_vel_std);
        double v_lat_bias = gnss_velocity_distribution(generator);
        lat_bias += v_lat_bias*(double)(AP_HAL::millis() - last_now)/1000.0;

        gnss_velocity_distribution = std::normal_distribution<double>(-lng_bias*lambda, gnss_vel_std);
        double v_lng_bias = gnss_velocity_distribution(generator);
        lng_bias += v_lng_bias*(double)(AP_HAL::millis() - last_now)/1000.0;

        gnss_velocity_distribution = std::normal_distribution<double>(-vertical_bias*lambda, gnss_vel_std);
        double v_vertical_bias = 1.5*gnss_velocity_distribution(generator);
        vertical_bias += v_vertical_bias*(double)(AP_HAL::millis() - last_now)/1000.0;

        last_now = AP_HAL::millis();
    }

    double getLatBias() const {
        return lat_bias;
    }

    double getLngBias() const {
        return lng_bias;
    }

    double getVerticalBias() const {
        return vertical_bias;
    }

    double lat_bias;
    double lng_bias;
    double vertical_bias;
    double gnss_vel_std;
    double lambda;
    uint32_t last_now;
};

class LowFrequencyNoise {
    std::vector<double> history;
    size_t max_history_size;

public:
    LowFrequencyNoise(size_t history_size) : max_history_size(history_size) {}

    double addAndGetSmoothedNoise(double noise) {
        if (history.size() >= max_history_size) {
            history.erase(history.begin());
        }
        history.push_back(noise);

        // Calculate the average
        double sum = 0.0;
        for (double n : history) {
            sum += n;
        }
        return sum / history.size();
    }
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
};

}

#endif  // HAL_SIM_GPS_ENABLED
