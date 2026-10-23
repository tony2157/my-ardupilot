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
  Simulated LadyBug LB5900 RF power sensor (ARRC), see SIM_LB5900.h
*/
#include "SIM_config.h"

#if AP_SIM_LB5900_ENABLED

#include "SIM_LB5900.h"

#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <string.h>

using namespace SITL;

const AP_Param::GroupInfo LB5900::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: LB5900 sim enable
    // @Description: Simulate an LB5900 RF power sensor on I2C bus 0, address 0x4C (ARRC)
    // @Values: 0:Absent,1:Present
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, LB5900, enable, 0),

    // @Param: PWR
    // @DisplayName: LB5900 sim power
    // @Description: Co-polarized power the simulated sensor reads, in dBm
    // @User: Advanced
    AP_GROUPINFO("PWR", 2, LB5900, power_dbm, -40.0f),

    // @Param: POL
    // @DisplayName: LB5900 sim polarization offset
    // @Description: Probe rotation about its boresight (MNT1_ARRC_POFF) that aligns the polarization
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("POL", 3, LB5900, pol_deg, 0.0f),

    // @Param: NSE
    // @DisplayName: LB5900 sim noise
    // @Description: Standard deviation of the simulated reading
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("NSE", 4, LB5900, noise_db, 0.05f),

    // @Param: FAULT
    // @DisplayName: LB5900 sim fault
    // @Description: Sensor fault to simulate. Reset happens once and sets the parameter back to 0
    // @Values: 0:None,1:SCPI not-a-number reply,2:Garbage reply,3:Reset (loses its configuration),4:Never ready
    // @User: Advanced
    AP_GROUPINFO("FAULT", 5, LB5900, fault, 0),

    AP_GROUPEND
};

LB5900::LB5900()
{
    AP_Param::setup_object_defaults(this, var_info);
}

float LB5900::reading_dbm()
{
    if (!poff_searched) {
        enum ap_var_type ptype;
        AP_Param *p = AP_Param::find("MNT1_ARRC_POFF", &ptype);
        if (p != nullptr && ptype == AP_PARAM_FLOAT) {
            poff = (AP_Float *)p;
        }
        poff_searched = true;
    }
    const float offset = (poff != nullptr) ? poff->get() : 0.0f;
    const float d = radians(offset - pol_deg.get());
    const float lin = sq(cosf(d)) + 0.001f * sq(sinf(d));
    // sum of uniforms, close enough to gaussian
    const float n = (rand_float() + rand_float() + rand_float()) * noise_db.get();
    return power_dbm.get() + 10.0f * log10f(lin) + n;
}

void LB5900::command(const char *cmd)
{
    if (strncmp(cmd, "SYST:PRES", 9) == 0) {
        continuous = false;
    } else if (strcmp(cmd, "INIT:CONT ON") == 0) {
        continuous = true;
    } else if (strcmp(cmd, "FETCh?") == 0) {
        output_len = 0;
        if (!continuous || Fault(fault.get()) == Fault::NOT_READY) {
            return;
        }
        switch (Fault(fault.get())) {
        case Fault::SENTINEL:
            output_len = snprintf(output, sizeof(output), "9.91E+37\n");
            break;
        case Fault::GARBAGE:
            output_len = snprintf(output, sizeof(output), "ERR -113\n");
            break;
        default:
            output_len = snprintf(output, sizeof(output), "%+.8E\n", double(reading_dbm()));
            break;
        }
    }
}

int LB5900::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (!enable) {
        return -1;
    }
    if (Fault(fault.get()) == Fault::RESET) {
        continuous = false;
        output_len = 0;
        fault.set(0);
    }
    for (uint8_t i = 0; i < data->nmsgs; i++) {
        auto &msg = data->msgs[i];
        if (msg.flags == I2C_M_RD) {
            if (next_read == 0x06) {
                // status and length: bit 4 = output available, then the length big-endian
                if (msg.len >= 4) {
                    msg.buf[0] = (output_len > 0) ? 0x10 : 0x00;
                    msg.buf[1] = 0;
                    msg.buf[2] = 0;
                    msg.buf[3] = output_len;
                }
            } else if (next_read == 0x0C) {
                const uint16_t n = MIN(msg.len, uint16_t(output_len));
                memcpy(msg.buf, output, n);
                output_len = 0;
            } else {
                return -1;
            }
            continue;
        }
        // write: header byte, 3 length bytes, then a command terminated by a zero for 0x06
        if (msg.len < 4) {
            return -1;
        }
        next_read = msg.buf[0];
        if (next_read == 0x06 && msg.len > 4) {
            char cmd[64] {};
            memcpy(cmd, &msg.buf[4], MIN(uint16_t(msg.len - 4), uint16_t(sizeof(cmd) - 1)));
            command(cmd);
        }
    }
    return 0;
}

#endif  // AP_SIM_LB5900_ENABLED
