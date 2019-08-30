#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    AP_GROUPINFO("USER_SENSOR1", 3, UserParameters, user_sensor1, 0),
    AP_GROUPINFO("USER_SENSOR2", 4, UserParameters, user_sensor2, 0),
    AP_GROUPINFO("USER_SENSOR3", 5, UserParameters, user_sensor3, 0),
    AP_GROUPINFO("USER_SENSOR4", 6, UserParameters, user_sensor4, 0),
    AP_GROUPINFO("USER_SENSOR5", 7, UserParameters, user_sensor5, 0),
    AP_GROUPINFO("USER_SENSOR6", 8, UserParameters, user_sensor6, 0),
    AP_GROUPINFO("USER_SENSOR7", 9, UserParameters, user_sensor7, 0),
    AP_GROUPINFO("USER_SENSOR8", 10, UserParameters, user_sensor8, 0),
    
    AP_GROUPEND
};
