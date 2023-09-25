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

    // ARRC ARRC_SDR params
    AP_GROUPINFO("_SDR_FREQ", 0, UserParameters, ARRC_SDR_dfreq, 3070),
    AP_GROUPINFO("_SDR_NPULSE", 1, UserParameters, ARRC_SDR_davg, 1),
    AP_GROUPINFO("_SDR_MODE", 2, UserParameters, ARRC_SDR_dmode, 0),
    AP_GROUPINFO("_SDR_PRF", 3, UserParameters, ARRC_SDR_dprf, 5000),
    
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
