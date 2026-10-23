#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Mount_Params {

public:

    static const struct AP_Param::GroupInfo var_info[];

    AP_Mount_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Params);

    AP_Int8     type;               // mount type (see Mount::Type enum)
    AP_Int8     default_mode;       // default mode on startup and when control is returned from autopilot
    AP_Int16    rc_rate_max;        // Pilot rate control's maximum rate.  Set to zero to use angle control
    AP_Int16    roll_angle_min;     // roll angle min in degrees
    AP_Int16    roll_angle_max;     // roll angle max in degrees
    AP_Int16    pitch_angle_min;    // pitch angle min in degrees
    AP_Int16    pitch_angle_max;    // pitch angle max in degrees
    AP_Int16    yaw_angle_min;      // yaw angle min in degrees
    AP_Int16    yaw_angle_max;      // yaw angle max in degrees

    AP_Vector3f retract_angles;     // retracted position in degrees. vector.x = roll vector.y = pitch, vector.z=yaw
    AP_Vector3f neutral_angles;     // neutral position in degrees.  vector.x = roll vector.y = pitch, vector.z=yaw

    AP_Float    roll_stb_lead;      // ARRC mod: repurposed as MNT1_ARRC_AZTH, AUT frame azimuth in degrees
    AP_Float    pitch_stb_lead;     // ARRC mod: repurposed as MNT1_ARRC_GMODE, probe orientation mode
    AP_Int8     sysid_default;      // target sysid for mount to follow
    AP_Int32    dev_id;             // Device id taking into account bus
    AP_Int8     options;            // mount options bitmask

    // ARRC params
    AP_Float    ARRC_elev;          // AUT elevation
    AP_Float    ARRC_z_offset;      // Gimbal offset from the vehicle origin along the body up axis in meters (down is negative)
    AP_Float    ARRC_x_offset;      // Gimbal offset from the vehicle origin along the body forward axis in meters
    AP_Float    ARRC_y_offset;      // Gimbal offset from the vehicle origin along the body right axis in meters
    AP_Float    ARRC_pol_offset;    // Polarization offset about the probe boresight found by the gimbal alignment, in degrees
    AP_Float    ARRC_lead;          // Position lead time compensating the gimbal command latency, in seconds
    AP_Float    ARRC_slew;          // Rate limit of the ARRC modes' angle commands, in degrees per second
};
