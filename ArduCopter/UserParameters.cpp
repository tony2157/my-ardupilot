#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    // AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    // AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    // AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    AP_GROUPINFO("_SENSOR1", 0, UserParameters, _sensor1, 0),
    AP_GROUPINFO("_SENSOR2", 1, UserParameters, _sensor2, 0),
    AP_GROUPINFO("_SENSOR3", 2, UserParameters, _sensor3, 0),
    AP_GROUPINFO("_SENSOR4", 3, UserParameters, _sensor4, 0),
    AP_GROUPINFO("_SENSOR5", 4, UserParameters, _sensor5, 0),
    AP_GROUPINFO("_SENSOR6", 5, UserParameters, _sensor6, 0),
    AP_GROUPINFO("_SENSOR7", 6, UserParameters, _sensor7, 0),
    AP_GROUPINFO("_SENSOR8", 7, UserParameters, _sensor8, 0),
    AP_GROUPINFO("_WV_MINRLL", 8, UserParameters, wind_vane_min_roll, 0),
    AP_GROUPINFO("_WV_FRATE", 9, UserParameters, wind_vane_fine_rate, 1),
    AP_GROUPINFO("_WV_FGAIN", 10, UserParameters, wind_vane_fine_gain, 1),
    AP_GROUPINFO("_WV_CUTOFF", 11, UserParameters, wind_vane_cutoff, 0.06f),
    AP_GROUPINFO("_WV_WSA", 12, UserParameters, wind_vane_wsA, 32.8f),
    AP_GROUPINFO("_WV_WSB", 13, UserParameters, wind_vane_wsB, -4.5f),
    
    AP_GROUPEND
};
