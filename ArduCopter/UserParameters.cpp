#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    // AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    // AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    // AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    // ARRC LB680A params
    AP_GROUPINFO("_LB_DFREQ", 0, UserParameters, LB680A_dfreq, 3070),
    AP_GROUPINFO("_LB_DAVG", 1, UserParameters, LB680A_davg, 20),
    AP_GROUPINFO("_LB_DMODE", 2, UserParameters, LB680A_dmode, 1),
    
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
