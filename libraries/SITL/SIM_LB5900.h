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
  Simulated LadyBug LB5900 RF power sensor (ARRC): SCPI over I2C, bus 0 address 0x4C.

  SIM_LB59_ENABLE 1 makes it answer. The power follows the probe polarization offset MNT1_ARRC_POFF against the
  simulated true offset SIM_LB59_POL (cross-polarization 30 dB down), so the gimbal alignment can be flown.
  SIM_LB59_FAULT injects sensor faults.
*/
#pragma once

#include "SIM_config.h"

#if AP_SIM_LB5900_ENABLED

#include "SIM_I2C.h"
#include <AP_Param/AP_Param.h>

namespace SITL {

// a SIM member (for its parameters); SIM_I2C.cpp puts it on the bus
class LB5900
{
public:
    LB5900();

    static const AP_Param::GroupInfo var_info[];

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data);

    enum class Fault : uint8_t {
        NONE = 0,
        SENTINEL = 1,       // FETCh? answers the SCPI not-a-number code 9.91E+37
        GARBAGE = 2,        // FETCh? answers text that is not a number
        RESET = 3,          // the sensor resets once (brownout) and loses its configuration
        NOT_READY = 4,      // never has a reading ready
    };

private:
    AP_Int8 enable;
    AP_Float power_dbm;     // co-polarized power
    AP_Float pol_deg;       // true polarization offset of the probe
    AP_Float noise_db;      // standard deviation of the reading
    AP_Int8 fault;

    void command(const char *cmd);
    float reading_dbm();

    bool continuous;        // INIT:CONT ON received since the last reset
    uint8_t next_read;      // header of the last write: 0x06 status and length, 0x0C output buffer
    char output[40];
    uint8_t output_len;
    AP_Float *poff;         // MNT1_ARRC_POFF
    bool poff_searched;
};

}  // namespace SITL

#endif  // AP_SIM_LB5900_ENABLED
