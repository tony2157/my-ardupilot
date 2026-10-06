#include "AP_Mount_Params.h"
#include <GCS_MAVLink/GCS_MAVLink.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_Mount_Params::var_info[] = {

    // 0 should not be used

    // @Param: _TYPE
    // @DisplayName: Mount Type
    // @Description: Mount Type
    // @SortValues: AlphabeticalZeroAtTop
    // @Values: 0:None, 1:Servo, 2:3DR Solo, 3:Alexmos Serial, 4:SToRM32 MAVLink, 5:SToRM32 Serial, 6:Gremsy, 7:BrushlessPWM, 8:Siyi, 9:Scripting, 10:Xacti, 11:Viewpro, 12:Topotek, 13:CADDX
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_Mount_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _DEFLT_MODE
    // @DisplayName: Mount default operating mode
    // @Description: Mount default operating mode on startup and after control is returned from autopilot
    // @Values: 0:Retracted,1:Neutral,2:MavLink Targeting,3:RC Targeting,4:GPS Point,5:SysID Target,6:Home Location
    // @User: Standard
    AP_GROUPINFO("_DEFLT_MODE", 2, AP_Mount_Params, default_mode, MAV_MOUNT_MODE_RC_TARGETING),

    // @Param: _RC_RATE
    // @DisplayName: Mount RC Rate
    // @Description: Pilot rate control's maximum rate.  Set to zero to use angle control
    // @Units: deg/s
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_RC_RATE", 3, AP_Mount_Params, rc_rate_max, 0),

    // @Param: _ROLL_MIN
    // @DisplayName: Mount Roll angle minimum
    // @Description: Mount Roll angle minimum
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ROLL_MIN", 4, AP_Mount_Params, roll_angle_min, -30),

    // @Param: _ROLL_MAX
    // @DisplayName: Mount Roll angle maximum
    // @Description: Mount Roll angle maximum
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ROLL_MAX", 5, AP_Mount_Params, roll_angle_max, 30),

    // @Param: _PITCH_MIN
    // @DisplayName: Mount Pitch angle minimum
    // @Description: Mount Pitch angle minimum
    // @Units: deg
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_PITCH_MIN", 6, AP_Mount_Params, pitch_angle_min, -90),

    // @Param: _PITCH_MAX
    // @DisplayName: Mount Pitch angle maximum
    // @Description: Mount Pitch angle maximum
    // @Units: deg
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_PITCH_MAX", 7, AP_Mount_Params, pitch_angle_max, 20),

    // @Param: _YAW_MIN
    // @DisplayName: Mount Yaw angle minimum
    // @Description: Mount Yaw angle minimum
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_YAW_MIN", 8, AP_Mount_Params, yaw_angle_min,  -180),

    // @Param: _YAW_MAX
    // @DisplayName: Mount Yaw angle maximum
    // @Description: Mount Yaw angle maximum
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_YAW_MAX", 9, AP_Mount_Params, yaw_angle_max,  180),

    // @Param: _RETRACT_X
    // @DisplayName: Mount roll angle when in retracted position
    // @Description: Mount roll angle when in retracted position
    // @Units: deg
    // @Range: -180.0 180.0
    // @Increment: 1
    // @User: Standard

    // @Param: _RETRACT_Y
    // @DisplayName: Mount pitch angle when in retracted position
    // @Description: Mount pitch angle when in retracted position
    // @Units: deg
    // @Range: -180.0 180.0
    // @Increment: 1
    // @User: Standard

    // @Param: _RETRACT_Z
    // @DisplayName: Mount yaw angle when in retracted position
    // @Description: Mount yaw angle when in retracted position
    // @Units: deg
    // @Range: -180.0 180.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_RETRACT", 10, AP_Mount_Params, retract_angles, 0),

    // @Param: _NEUTRAL_X
    // @DisplayName: Mount roll angle when in neutral position
    // @Description: Mount roll angle when in neutral position
    // @Units: deg
    // @Range: -180.0 180.0
    // @Increment: 1
    // @User: Standard

    // @Param: _NEUTRAL_Y
    // @DisplayName: Mount pitch angle when in neutral position
    // @Description: Mount pitch angle when in neutral position
    // @Units: deg
    // @Range: -180.0 180.0
    // @Increment: 1
    // @User: Standard

    // @Param: _NEUTRAL_Z
    // @DisplayName: Mount yaw angle when in neutral position
    // @Description: Mount yaw angle when in neutral position
    // @Units: deg
    // @Range: -180.0 180.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_NEUTRAL", 11, AP_Mount_Params, neutral_angles, 0),

    // @Param: _ARRC_AZTH
    // @DisplayName: ARRC AUT frame azimuth
    // @Description: Azimuth the probe faces when it looks at the AUT along the AUT boresight (AUT boresight azimuth + 180). Sets the probe yaw in mode 1 and the AUT reference frame in modes 2 and 3
    // @Units: deg
    // @Range: -180 180
    // @Increment: .1
    // @User: Standard
    AP_GROUPINFO("_ARRC_AZTH", 12, AP_Mount_Params, roll_stb_lead, 0.0f),

    // @Param: _ARRC_GMODE
    // @DisplayName: ARRC gimbal mode
    // @Description: How the probe is oriented while pointing at a location (ROI, home or sysid target)
    // @Values: 0:ArduPilot pointing, 1:Probe plane parallel to AUT plane, 2:Hpol aligned, 3:Vpol aligned
    // @User: Standard
    AP_GROUPINFO("_ARRC_GMODE", 13, AP_Mount_Params, pitch_stb_lead, 0.0f),

    // @Param: _ARRC_ELEV
    // @DisplayName: AUT elevation
    // @Description: Elevation angle of the AUT in degrees
    // @Units: deg
    // @Range: 0.0 90
    // @Increment: .1
    // @User: Standard
    AP_GROUPINFO("_ARRC_ELEV", 14, AP_Mount_Params, ARRC_elev, 0.0f),

    // @Param: _ARRC_ZOFF
    // @DisplayName: ARRC gimbal up offset
    // @Description: Offset of the gimbal rotation center from the vehicle origin (IMU) along the body up axis. Down is negative. Rotated with the vehicle attitude
    // @Units: m
    // @Range: -2 2
    // @Increment: .01
    // @User: Standard
    AP_GROUPINFO("_ARRC_ZOFF", 15, AP_Mount_Params, ARRC_z_offset, 0.0f),

    // @Param: _SYSID_DFLT
    // @DisplayName: Mount Target sysID
    // @Description: Default Target sysID for the mount to point to
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_SYSID_DFLT", 16, AP_Mount_Params, sysid_default, 0),

    // @Param: _DEVID
    // @DisplayName: Mount Device ID
    // @Description: Mount device ID, taking into account its type, bus and instance
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_DEVID", 17, AP_Mount_Params, dev_id, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

    // @Param: _OPTIONS
    // @DisplayName: Mount options
    // @Description: Mount options bitmask
    // @Bitmask: 0:RC lock state from previous mode
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 18, AP_Mount_Params, options, 0),

    // @Param: _ARRC_XOFF
    // @DisplayName: ARRC gimbal forward offset
    // @Description: Offset of the gimbal rotation center from the vehicle origin (IMU) along the body forward axis. Rotated with the vehicle attitude
    // @Units: m
    // @Range: -2 2
    // @Increment: .01
    // @User: Standard
    AP_GROUPINFO("_ARRC_XOFF", 19, AP_Mount_Params, ARRC_x_offset, 0.0f),

    // @Param: _ARRC_YOFF
    // @DisplayName: ARRC gimbal right offset
    // @Description: Offset of the gimbal rotation center from the vehicle origin (IMU) along the body right axis. Rotated with the vehicle attitude
    // @Units: m
    // @Range: -2 2
    // @Increment: .01
    // @User: Standard
    AP_GROUPINFO("_ARRC_YOFF", 20, AP_Mount_Params, ARRC_y_offset, 0.0f),

    // @Param: _ARRC_POFF
    // @DisplayName: ARRC polarization offset
    // @Description: Rotation of the probe about its boresight applied in modes 2 and 3. Set by the gimbal alignment (aux switch) and kept across reboots; the switch clears it to 0
    // @Units: deg
    // @Range: -90 90
    // @Increment: .01
    // @User: Standard
    AP_GROUPINFO("_ARRC_POFF", 21, AP_Mount_Params, ARRC_pol_offset, 0.0f),

    // @Param: _ARRC_LEAD
    // @DisplayName: ARRC position lead time
    // @Description: The probe position used for the gimbal angles is predicted this far ahead using the vehicle velocity, to compensate the gimbal command latency. 0 disables
    // @Units: s
    // @Range: 0 0.3
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("_ARRC_LEAD", 22, AP_Mount_Params, ARRC_lead, 0.0f),

    AP_GROUPEND
};

AP_Mount_Params::AP_Mount_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
