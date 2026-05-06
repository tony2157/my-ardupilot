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

    //CASS custom parameters publish
    // Weather sensors
    AP_GROUPINFO("_SENSOR1", 0, UserParameters, _sensor1, 0),
    AP_GROUPINFO("_SENSOR2", 1, UserParameters, _sensor2, 0),
    AP_GROUPINFO("_SENSOR3", 2, UserParameters, _sensor3, 0),
    AP_GROUPINFO("_SENSOR4", 3, UserParameters, _sensor4, 0),
    AP_GROUPINFO("_SENSOR5", 4, UserParameters, _sensor5, 0),
    AP_GROUPINFO("_SENSOR6", 5, UserParameters, _sensor6, 0),
    AP_GROUPINFO("_SENSOR7", 6, UserParameters, _sensor7, 0),
    AP_GROUPINFO("_SENSOR8", 7, UserParameters, _sensor8, 0),
    // Wind Vane
    AP_GROUPINFO("_WV_CUTOFF", 8, UserParameters, wind_vane_cutoff, 0.06f),
    AP_GROUPINFO("_WV_WSA", 9, UserParameters, wind_vane_wsA, 30.0f),
    AP_GROUPINFO("_WV_WSB", 10, UserParameters, wind_vane_wsB, 9.0f),
    AP_GROUPINFO("_WV_SPDTOL", 11, UserParameters, wind_vane_spd_tol, 19.0f),
    AP_GROUPINFO("_WV_RTLEN", 12, UserParameters, wind_vane_enabled, 1.0f),
    AP_GROUPINFO("_WV_FS", 13, UserParameters, wind_vane_fs, 10.0f),
    AP_GROUPINFO("_WV_OFFSET", 14, UserParameters, wind_vane_offset, 0.0f),
    // VPBatt
    AP_GROUPINFO("_VPBATT_EN", 15, UserParameters, vpbatt_enabled, 1.0f),
    AP_GROUPINFO("_VPBATT_RES", 16, UserParameters, vpbatt_reserve, 30.0f),
    AP_GROUPINFO("_VPBATT_WH", 17, UserParameters, vpbatt_wh, 89.0f),
    AP_GROUPINFO("_MAX_CURR", 21, UserParameters, batt_max_curr, 45.0f),
    AP_GROUPINFO("_CURR_TOUT", 22, UserParameters, max_curr_timeout, 5.0f),
    // Mission Auto-generator
    AP_GROUPINFO("_AUTOVP_ALT", 18, UserParameters, autovp_max_altitude, 120.0f),
    // GPS-based Lidar activation
    AP_GROUPINFO("_LIDAR_ALT", 19, UserParameters, gpslidar_alt, 80.0f),
    AP_GROUPINFO("_LIDAR_HUM", 20, UserParameters, gpslidar_hum, 90.0f),
    // BLISS temp sensor coefficients
    AP_GROUPINFO("_SEN_A1", 23, UserParameters, _senA_c1, 10104.8989f),
    AP_GROUPINFO("_SEN_A2", 24, UserParameters, _senA_c2, 2620.50421f),
    AP_GROUPINFO("_SEN_A3", 25, UserParameters, _senA_c3, 0.0f),
    AP_GROUPINFO("_SEN_A4", 26, UserParameters, _senA_c4, 1.48891207f),
    AP_GROUPINFO("_SEN_B1", 27, UserParameters, _senB_c1, 10104.8989f),
    AP_GROUPINFO("_SEN_B2", 28, UserParameters, _senB_c2, 2620.50421f),
    AP_GROUPINFO("_SEN_B3", 29, UserParameters, _senB_c3, 0.0f),
    AP_GROUPINFO("_SEN_B4", 30, UserParameters, _senB_c4, 1.48891207f),
    AP_GROUPINFO("_SEN_C1", 31, UserParameters, _senC_c1, 10104.8989f),
    AP_GROUPINFO("_SEN_C2", 32, UserParameters, _senC_c2, 2620.50421f),
    AP_GROUPINFO("_SEN_C3", 33, UserParameters, _senC_c3, 0.0f),
    AP_GROUPINFO("_SEN_C4", 34, UserParameters, _senC_c4, 1.48891207f),
    // BLISS RH sensor coefficients are nested in sub-groups: the
    // parent table would otherwise exceed AP_Param's 64-entry hard
    // limit. Each block collapses 12 entries into one sub-group entry.
    // User-visible names remain USR_RH_A1..USR_RH_C12 unchanged.
    AP_SUBGROUPINFO(_RHA, "_RH_A", 35, UserParameters, RHACoeffs),
    AP_SUBGROUPINFO(_RHB, "_RH_B", 36, UserParameters, RHBCoeffs),
    AP_SUBGROUPINFO(_RHC, "_RH_C", 37, UserParameters, RHCCoeffs),

    AP_GROUPEND
};

const AP_Param::GroupInfo RHACoeffs::var_info[] = {
    AP_GROUPINFO("1",  0,  RHACoeffs, c1,  12386.8608f),
    AP_GROUPINFO("2",  1,  RHACoeffs, c2,  17022.2288f),
    AP_GROUPINFO("3",  2,  RHACoeffs, c3,  1779786.3116f),
    AP_GROUPINFO("4",  3,  RHACoeffs, c4,  1076288.8976f),
    AP_GROUPINFO("5",  4,  RHACoeffs, c5,  -41865.2086f),
    AP_GROUPINFO("6",  5,  RHACoeffs, c6,  -5943.8831f),
    AP_GROUPINFO("7",  6,  RHACoeffs, c7,  -6298.4062f),
    AP_GROUPINFO("8",  7,  RHACoeffs, c8,  -4039.3164f),
    AP_GROUPINFO("9",  8,  RHACoeffs, c9,  230.8740f),
    AP_GROUPINFO("10", 9,  RHACoeffs, c10, 12.9706f),
    AP_GROUPINFO("11", 10, RHACoeffs, c11, 12.8721f),
    AP_GROUPINFO("12", 11, RHACoeffs, c12, 3.1870f),
    AP_GROUPEND
};

const AP_Param::GroupInfo RHBCoeffs::var_info[] = {
    AP_GROUPINFO("1",  0,  RHBCoeffs, c1,  12169.3356f),
    AP_GROUPINFO("2",  1,  RHBCoeffs, c2,  7991.2309f),
    AP_GROUPINFO("3",  2,  RHBCoeffs, c3,  1771509.7306f),
    AP_GROUPINFO("4",  3,  RHBCoeffs, c4,  508948.7724f),
    AP_GROUPINFO("5",  4,  RHBCoeffs, c5,  -31145.3196f),
    AP_GROUPINFO("6",  5,  RHBCoeffs, c6,  -5933.6122f),
    AP_GROUPINFO("7",  6,  RHBCoeffs, c7,  -5890.1048f),
    AP_GROUPINFO("8",  7,  RHBCoeffs, c8,  -435.1848f),
    AP_GROUPINFO("9",  8,  RHBCoeffs, c9,  195.9107f),
    AP_GROUPINFO("10", 9,  RHBCoeffs, c10, 13.3835f),
    AP_GROUPINFO("11", 10, RHBCoeffs, c11, 11.4385f),
    AP_GROUPINFO("12", 11, RHBCoeffs, c12, -2.5781f),
    AP_GROUPEND
};

const AP_Param::GroupInfo RHCCoeffs::var_info[] = {
    AP_GROUPINFO("1",  0,  RHCCoeffs, c1,  13123.4403f),
    AP_GROUPINFO("2",  1,  RHCCoeffs, c2,  3639.7364f),
    AP_GROUPINFO("3",  2,  RHCCoeffs, c3,  1922025.0525f),
    AP_GROUPINFO("4",  3,  RHCCoeffs, c4,  215521.2076f),
    AP_GROUPINFO("5",  4,  RHCCoeffs, c5,  -42127.4389f),
    AP_GROUPINFO("6",  5,  RHCCoeffs, c6,  -6363.9524f),
    AP_GROUPINFO("7",  6,  RHCCoeffs, c7,  -5797.4928f),
    AP_GROUPINFO("8",  7,  RHCCoeffs, c8,  1703.2685f),
    AP_GROUPINFO("9",  8,  RHCCoeffs, c9,  226.9615f),
    AP_GROUPINFO("10", 9,  RHCCoeffs, c10, 15.6081f),
    AP_GROUPINFO("11", 10, RHCCoeffs, c11, 9.7073f),
    AP_GROUPINFO("12", 11, RHCCoeffs, c12, -6.1066f),
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
