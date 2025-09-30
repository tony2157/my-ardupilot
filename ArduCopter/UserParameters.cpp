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
    
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
