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

    // ARRC LB5900 params
    AP_GROUPINFO("_LB_ADDR", 0, UserParameters, lb5900_addr, 76),
    AP_GROUPINFO("_LB_FREQ", 1, UserParameters, lb5900_freq, 3000),
    AP_GROUPINFO("_LB_AVG_CNT", 2, UserParameters, lb5900_avg_cnt, 10),
    AP_GROUPINFO("_LB_RATE", 3, UserParameters, lb5900_mrate, 1),
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
    // @Description: Onboard SDR safety and arming options
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
