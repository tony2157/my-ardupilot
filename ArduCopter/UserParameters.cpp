#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    // AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    // AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    // AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    // ARRC LB5900 USB/I2C power sensor (libraries/AP_ARRC_LB5900), read at boot by init_ARRC_lb5900()

    // @Param: _LB_ADDR
    // @DisplayName: LB5900 I2C address
    // @Description: 7-bit I2C address of the LB5900 power sensor, in decimal (76 = 0x4C, the sensor default)
    // @Range: 8 119
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_LB_ADDR", 0, UserParameters, lb5900_addr, 76),

    // @Param: _LB_FREQ
    // @DisplayName: LB5900 measurement frequency
    // @Description: Carrier frequency the LB5900 corrects its power reading for. Set it to the frequency of the antenna under test
    // @Units: MHz
    // @Range: 1 18000
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_LB_FREQ", 1, UserParameters, lb5900_freq, 3000),

    // @Param: _LB_AVG_CNT
    // @DisplayName: LB5900 averaging count
    // @Description: Readings the LB5900 averages per measurement (SENS:AVER:COUN). Each reading takes 38.4 ms at rate 0, 19.6 ms at 1, 1.6 ms at 3; rate 2 forces 1
    // @Range: 1 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_LB_AVG_CNT", 2, UserParameters, lb5900_avg_cnt, 10),

    // @Param: _LB_RATE
    // @DisplayName: LB5900 measurement rate
    // @Description: LB5900 measurement rate mode (SENS:MRAT). The driver polls the sensor at the resulting period: time per reading times the averaging count
    // @Values: 0:Normal (38.4 ms per reading),1:Double (19.6 ms),2:Fast (3.2 ms without averaging),3:Super (1.6 ms)
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_LB_RATE", 3, UserParameters, lb5900_mrate, 1),

    // @Param: _LB_BUS
    // @DisplayName: LB5900 I2C bus
    // @Description: I2C bus the LB5900 is connected to (0 is the Cube's external I2C 1)
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_LB_BUS", 4, UserParameters, lb5900_busId, 0),

    // @Param: _RF_SRC
    // @DisplayName: RF power source
    // @Description: RF power sensor used for logging and streaming. Auto uses the LB5900 until the companion computer and its SDR are detected, then disables the LB5900 until reboot. Both keeps the LB5900 and the SDR running together (e.g. behind a power splitter).
    // @Values: 0:Auto,1:LB5900 only,2:SDR only,3:Both
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_RF_SRC", 5, UserParameters, rf_src, 0),

    // @Param: _SDR_OPT
    // @DisplayName: Onboard SDR options
    // @Description: Onboard SDR safety and arming options. Bit 2 refuses arming unless the companion computer (a heartbeat of type ONBOARD_CONTROLLER from component 191 with the vehicle's MAV_SYSID) reports an SDR with the device present and a measurement younger than 2 s; it is skipped when the Camera bit of ARMING_CHECK is cleared.
    // @Bitmask: 0:Stop SDR TX on disarm,1:Stop SDR TX on failsafe,2:Require SDR measurements to arm
    // @User: Standard
    AP_GROUPINFO("_SDR_OPT", 6, UserParameters, sdr_options, 3),

    // @Param: _SDR_TOUT
    // @DisplayName: Onboard SDR link timeout
    // @Description: Time without companion computer heartbeat or SDR status before the SDR is considered lost
    // @Units: s
    // @Range: 1 10
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("_SDR_TOUT", 7, UserParameters, sdr_timeout, 3),
    
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
