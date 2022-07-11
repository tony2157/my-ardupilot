#define ALLOW_DOUBLE_MATH_FUNCTIONS
#include "AP_Mount_Backend.h"
#if HAL_MOUNT_ENABLED
#include <AP_AHRS/AP_AHRS.h>
//#include <GCS_MAVLink/GCS.h> // Enable this for debugging

extern const AP_HAL::HAL& hal;

//uint32_t _now = 0; // Enable this for debugging

// set_angle_targets - sets angle targets in degrees
void AP_Mount_Backend::set_angle_targets(float roll, float tilt, float pan)
{
    // set angle targets
    _angle_ef_target_rad_d.x = radians(roll);
    _angle_ef_target_rad_d.y = radians(tilt);
    _angle_ef_target_rad_d.z = radians(pan);

    // set the mode to mavlink targeting
    _frontend.set_mode(_instance, MAV_MOUNT_MODE_MAVLINK_TARGETING);
}

// ARRC set fixed yaw angle after antenna alignment
void AP_Mount_Backend::set_fixed_yaw_angle(float fixed_yaw)
{
    // set angle targets
    _state._roll_stb_lead = fixed_yaw;
    // set the mode to mavlink targeting
    //_frontend.set_mode(_instance, MAV_MOUNT_MODE_GPS_POINT);
}

// ARRC set fixed yaw angle after antenna alignment
void AP_Mount_Backend::set_RotM_offset(Matrix3d rotm_off)
{
    // set Rotation matrix offset
    _state._rotM_offset = rotm_off;
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount_Backend::set_roi_target(const struct Location &target_loc)
{
    // set the target gps location
    _state._roi_target = target_loc;
    _state._roi_target_set = true;

    // set the mode to GPS tracking mode
    _frontend.set_mode(_instance, MAV_MOUNT_MODE_GPS_POINT);
}

// set_sys_target - sets system that mount should attempt to point towards
void AP_Mount_Backend::set_target_sysid(uint8_t sysid)
{
    _state._target_sysid = sysid;

    // set the mode to sysid tracking mode
    _frontend.set_mode(_instance, MAV_MOUNT_MODE_SYSID_TARGET);
}

// process MOUNT_CONFIGURE messages received from GCS.  deprecated.
void AP_Mount_Backend::handle_mount_configure(const mavlink_mount_configure_t &packet)
{
    set_mode((MAV_MOUNT_MODE)packet.mount_mode);
    _state._stab_roll = packet.stab_roll;
    _state._stab_tilt = packet.stab_pitch;
    _state._stab_pan = packet.stab_yaw;
}

// process MOUNT_CONTROL messages received from GCS. deprecated.
void AP_Mount_Backend::handle_mount_control(const mavlink_mount_control_t &packet)
{
    control((int32_t)packet.input_a, (int32_t)packet.input_b, (int32_t)packet.input_c, _state._mode);
}

void AP_Mount_Backend::control(int32_t pitch_or_lat, int32_t roll_or_lon, int32_t yaw_or_alt, MAV_MOUNT_MODE mount_mode)
{
    _frontend.set_mode(_instance, mount_mode);

    // interpret message fields based on mode
    switch (_frontend.get_mode(_instance)) {
        case MAV_MOUNT_MODE_RETRACT:
        case MAV_MOUNT_MODE_NEUTRAL:
            // do nothing with request if mount is retracted or in neutral position
            break;

        // set earth frame target angles from mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            set_angle_targets(roll_or_lon*0.01f, pitch_or_lat*0.01f, yaw_or_alt*0.01f);
            break;

        // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
        case MAV_MOUNT_MODE_RC_TARGETING:
            // do nothing if pilot is controlling the roll, pitch and yaw
            break;

        // set lat, lon, alt position targets from mavlink message

        case MAV_MOUNT_MODE_GPS_POINT: {
            const Location target_location{
                pitch_or_lat,
                roll_or_lon,
                yaw_or_alt,
                Location::AltFrame::ABOVE_HOME
            };
            set_roi_target(target_location);
            break;
        }

        case MAV_MOUNT_MODE_HOME_LOCATION: {
            // set the target gps location
            _state._roi_target = AP::ahrs().get_home();
            _state._roi_target_set = true;
            break;
        }

        default:
            // do nothing
            break;
    }
}

void AP_Mount_Backend::rate_input_rad(float &out, const RC_Channel *chan, float min, float max) const
{
    if ((chan == nullptr) || (chan->get_radio_in() == 0)) {
        return;
    }
    out += chan->norm_input_dz() * 0.0001f * _frontend._joystick_speed;
    out = constrain_float(out, radians(min*0.01f), radians(max*0.01f));
}

// update_targets_from_rc - updates angle targets using input from receiver
void AP_Mount_Backend::update_targets_from_rc()
{
    const RC_Channel *roll_ch = rc().channel(_state._roll_rc_in - 1);
    const RC_Channel *tilt_ch = rc().channel(_state._tilt_rc_in - 1);
    const RC_Channel *pan_ch = rc().channel(_state._pan_rc_in - 1);

    // if joystick_speed is defined then pilot input defines a rate of change of the angle
    if (_frontend._joystick_speed) {
        // allow pilot position input to come directly from an RC_Channel
        rate_input_rad(_angle_ef_target_rad.x,
                       roll_ch,
                       _state._roll_angle_min,
                       _state._roll_angle_max);
        rate_input_rad(_angle_ef_target_rad.y,
                       tilt_ch,
                       _state._tilt_angle_min,
                       _state._tilt_angle_max);
        rate_input_rad(_angle_ef_target_rad.z,
                       pan_ch,
                       _state._pan_angle_min,
                       _state._pan_angle_max);
    } else {
        // allow pilot rate input to come directly from an RC_Channel
        if ((roll_ch != nullptr) && (roll_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.x = angle_input_rad(roll_ch, _state._roll_angle_min, _state._roll_angle_max);
        }
        if ((tilt_ch != nullptr) && (tilt_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.y = angle_input_rad(tilt_ch, _state._tilt_angle_min, _state._tilt_angle_max);
        }
        if ((pan_ch != nullptr) && (pan_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.z = angle_input_rad(pan_ch, _state._pan_angle_min, _state._pan_angle_max);
        }
    }
}

// returns the angle (radians) that the RC_Channel input is receiving
float AP_Mount_Backend::angle_input_rad(const RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(((rc->norm_input_ignore_trim() + 1.0f) * 0.5f * (angle_max - angle_min) + angle_min)*0.01f);
}

bool AP_Mount_Backend::calc_angle_to_roi_target(Vector3f& angles_to_target_rad,
                                                bool calc_tilt,
                                                bool calc_pan,
                                                bool relative_pan) const
{
    if (!_state._roi_target_set) {
        return false;
    }
    return calc_angle_to_location(_state._roi_target, angles_to_target_rad, calc_tilt, calc_pan, relative_pan);
}

bool AP_Mount_Backend::calc_angle_to_roi_target_d(Vector3d& angles_to_target_rad,
                                                bool calc_tilt,
                                                bool calc_pan,
                                                bool relative_pan) const
{
    if (!_state._roi_target_set) {
        return false;
    }
    return calc_angle_to_location_d(_state._roi_target, angles_to_target_rad, calc_tilt, calc_pan, relative_pan);
}

bool AP_Mount_Backend::calc_angle_to_sysid_target(Vector3f& angles_to_target_rad,
                                                  bool calc_tilt,
                                                  bool calc_pan,
                                                  bool relative_pan) const
{
    if (!_state._target_sysid_location_set) {
        return false;
    }
    if (!_state._target_sysid) {
        return false;
    }
    return calc_angle_to_location(_state._target_sysid_location,
                                  angles_to_target_rad,
                                  calc_tilt,
                                  calc_pan,
                                  relative_pan);
}

bool AP_Mount_Backend::calc_angle_to_sysid_target_d(Vector3d& angles_to_target_rad,
                                                  bool calc_tilt,
                                                  bool calc_pan,
                                                  bool relative_pan) const
{
    if (!_state._target_sysid_location_set) {
        return false;
    }
    if (!_state._target_sysid) {
        return false;
    }
    return calc_angle_to_location_d(_state._target_sysid_location,
                                  angles_to_target_rad,
                                  calc_tilt,
                                  calc_pan,
                                  relative_pan);
}

// calc_angle_to_location - calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
bool AP_Mount_Backend::calc_angle_to_location(const struct Location &target, Vector3f& angles_to_target_rad, bool calc_tilt, bool calc_pan, bool relative_pan) const
{
    Location current_loc;
    if (!AP::ahrs().get_position(current_loc)) {
        return false;
    }

    // Haversine formula
    float curr_lat = current_loc.lat*1.0e-7*M_PI/180.0;
    float tar_lat = target.lat*1.0e-7*M_PI/180.0;
    float delta_lat = tar_lat - curr_lat;
    float delta_lng = Location::diff_longitude(target.lng,current_loc.lng)*1.0e-7*M_PI/180.0;
    float a = sinf(delta_lat/2)*sinf(delta_lat/2) + cosf(curr_lat)*cosf(tar_lat)*sinf(delta_lng/2)*sinf(delta_lng/2);
    float target_distance = 2*RADIUS_OF_EARTH*atan2f(sqrt(a),sqrtf(1-a))*100.0; // in cm
    float y = sinf(delta_lng)*cosf(tar_lat);
    float x = cosf(curr_lat)*sinf(tar_lat) - sinf(curr_lat)*cosf(tar_lat)*cosf(delta_lng);

    int32_t target_alt_cm = 0;
    if (!target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm)) {
        return false;
    }
    int32_t current_alt_cm = 0;
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt_cm)) {
        return false;
    }

    float z = target_alt_cm - current_alt_cm;

    // initialise all angles to zero
    angles_to_target_rad.zero();

    // tilt calcs
    if (calc_tilt) {
        angles_to_target_rad.y = atan2f(z, target_distance);
    }

    // pan calcs
    if (calc_pan) {
        // calc absolute heading and then onvert to vehicle relative yaw
        angles_to_target_rad.z = atan2f(y, x);
        if (relative_pan) {
            angles_to_target_rad.z = wrap_PI(angles_to_target_rad.z - AP::ahrs().yaw);
        }
    }
    return true;
}

// calc_angle_to_location - calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
bool AP_Mount_Backend::calc_angle_to_location_d(const struct Location &target, Vector3d& angles_to_target_rad, bool calc_tilt, bool calc_pan, bool relative_pan) const
{
    Location current_loc;
    if (!AP::ahrs().get_position(current_loc)) {
        return false;
    }

    // Haversine formula
    double curr_lat = ((double)current_loc.lat)*1.0e-7*M_PI/180.0;
    double tar_lat = ((double)target.lat)*1.0e-7*M_PI/180.0;
    double delta_lat = tar_lat - curr_lat;
    double delta_lng = ((double)Location::diff_longitude(target.lng,current_loc.lng))*1.0e-7*M_PI/180.0;
    double target_distance = sin(delta_lat/2.0)*sin(delta_lat/2.0) + cos(curr_lat)*cos(tar_lat)*sin(delta_lng/2.0)*sin(delta_lng/2.0);

    // Compute distance to target
    target_distance = 2.0*RADIUS_OF_EARTH*atan2(sqrt(target_distance),sqrt(1.0-target_distance)); // in meters

    // Compute bearing to target
    double y = sin(delta_lng)*cos(tar_lat);
    double x = cos(curr_lat)*sin(tar_lat) - sin(curr_lat)*cos(tar_lat)*cos(delta_lng);
    double bearing = atan2(y, x);

    double fixed_yaw = (double)_state._roll_stb_lead*DEG_TO_RAD;

    // Compute target vector x-y components in the target's reference frame (NWU)
    y = target_distance*(sin(bearing)*cos(fixed_yaw) - cos(bearing)*sin(fixed_yaw)); // Aligned with West when fixed_yaw = 0
    x = -target_distance*(sin(bearing)*sin(fixed_yaw) + cos(bearing)*cos(fixed_yaw)); // Aligned with North when fixed_yaw = 0

    int32_t target_alt_cm = 0;
    if (!target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm)) {
        return false;
    }
    int32_t current_alt_cm = 0;
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt_cm)) {
        return false;
    }

    // Compute height difference (NWU)
    double z = (double)(current_alt_cm - target_alt_cm)/(100.0) + (double)_state._ARRC_z_offset; // in meters

    //Compute distance and slope wrt target
    float horzdist2target = current_loc.get_distance(target);
    float dist2target = sqrtf(horzdist2target*horzdist2target + (float)(z*z));
    float slope = 0;
    if(!is_zero(horzdist2target)) slope = fabsf((float)z/horzdist2target);

    // AUT elevation
    double el = _state._ARRC_elev*DEG_TO_RAD;

    // initialise all angles to zero
    angles_to_target_rad.zero();

    // Alexmos gimbal convention: Pitch down (+), Roll right (+), Yaw right (+)
    // This technique's convention: Pitch down (-), Roll right (-), Yaw right (+)
    // Position (x,y,z) is NWU convention with the AUT as the origin
    if(dist2target > 10){
        if(is_zero(_state._pitch_stb_lead) || (slope < 0.35f && el > 70)){

            // Original ArduPilot mode

            // tilt calcs
            angles_to_target_rad.y = atan2(-z, target_distance);

            // roll is leveled to the ground

            // pan calcs
            angles_to_target_rad.z = bearing;
            if (relative_pan) {
                // Convert to vehicle relative yaw
                angles_to_target_rad.z = wrap_180((angles_to_target_rad.z - (double)AP::ahrs().yaw)*RAD_TO_DEG)*DEG_TO_RAD;
            }
        }
        else if(is_zero(_state._pitch_stb_lead - 1)){

            // Point gimbal straight down without corrections

            // tilt calcs. Fixed 
            angles_to_target_rad.y = M_PI_2;
            
            // roll calcs. Fixed
            angles_to_target_rad.x = 0;

            // pan calcs. Fixed and equal to user param
            angles_to_target_rad.z = fixed_yaw;
            if (relative_pan) {
                angles_to_target_rad.z = wrap_180((angles_to_target_rad.z - (double)AP::ahrs().yaw)*RAD_TO_DEG)*DEG_TO_RAD;
            }
        }
        else if(is_zero(_state._pitch_stb_lead - 2)){

            // Hpol aligned mode

            double D = sqrt(x*x + y*y + z*z);
            double A = sqrt(x*x+z*z);

            Matrix3d RotM( -x/D,         -y/D,      -z/D,
                           -x*y/(A*D),   A/D,       -y*z/(A*D),
                           z/A,          0,         -x/A   );

            RotM = _state._rotM_offset*RotM;

            // tilt calcs = atan2(Reb(1,3),Reb(3,3))
            angles_to_target_rad.y = -1.0*atan2(RotM.a.z, RotM.c.z);
            
            // roll calcs = atan2(-Reb(2,3),sqrt(1-Reb(2,3)^2))
            angles_to_target_rad.x = -1.0*atan2(-RotM.b.z, sqrt(1.0 - RotM.b.z*RotM.b.z));

            // pan calcs = atan2(Reb(2,1),Reb(2,2))
            angles_to_target_rad.z = atan2f(RotM.b.x,RotM.b.y) + fixed_yaw;
            if (relative_pan) {
                angles_to_target_rad.z = wrap_180((angles_to_target_rad.z - (double)AP::ahrs().yaw)*RAD_TO_DEG)*DEG_TO_RAD;
            }
        }
        else if(is_zero(_state._pitch_stb_lead - 3)){

            // Vpol aligned mode

            double D = sqrt(x*x + y*y + z*z);
            double A = sqrt((x*x - z*z)*cos(el)*cos(el) + y*y + z*z - x*z*sin(2*el));
            double B = sqrt((x*x - z*z)*cos(2*el) + x*x + 2*y*y + z*z - 2*x*z*sin(2*el));

            Matrix3d RotM( -x/D,                                                    -y/D,                                                   -z/D,
                           y*cos(el)/A,                                             -(x*cos(el)-z*sin(el))/A,                               -y*sin(el)/A,
                           (M_SQRT2*((y*y*+z*z)*sin(el)-x*z*cos(el)))/(D*B),        -(M_SQRT2*y*(z*cos(el)+x*sin(el)))/(D*B),                M_SQRT2*((x*x+y*y)*cos(el)-x*z*sin(el))*B/(2*D*A*A)  );

            RotM =  _state._rotM_offset*RotM;

            // tilt calcs = atan2(Reb(1,3),Reb(3,3))
            angles_to_target_rad.y = -1.0*atan2(RotM.a.z, RotM.c.z);
            
            // roll calcs = atan2(-Reb(2,3),sqrt(1-Reb(2,3)^2))
            angles_to_target_rad.x = -1.0*atan2(-RotM.b.z, sqrt(1.0 - RotM.b.z*RotM.b.z));

            // pan calcs = atan2(Reb(2,1),Reb(2,2))
            angles_to_target_rad.z = atan2(RotM.b.x,RotM.b.y) + fixed_yaw;
            if (relative_pan) {
                angles_to_target_rad.z = wrap_180((angles_to_target_rad.z - (double)AP::ahrs().yaw)*RAD_TO_DEG)*DEG_TO_RAD;
            }
        }
    }

    // For debugging:
    // if (AP_HAL::millis() - _now > 3000){
    //     gcs().send_text(MAV_SEVERITY_INFO,"Target Dist: %5.2f",(float)target_distance);
    //     gcs().send_text(MAV_SEVERITY_INFO,"Bearing: %5.2f",(float)bearing);
    //     gcs().send_text(MAV_SEVERITY_INFO,"Target X: %5.2f",(float)x);
    //     gcs().send_text(MAV_SEVERITY_INFO,"Target Y: %5.2f",(float)y);
    //     gcs().send_text(MAV_SEVERITY_INFO,"Target Z: %5.2f",(float)z);
    //     gcs().send_text(MAV_SEVERITY_INFO,"Pitch: %5.2f",(float)angles_to_target_rad.y*RAD_TO_DEG);
    //     gcs().send_text(MAV_SEVERITY_INFO,"Roll: %5.2f",(float)angles_to_target_rad.x*RAD_TO_DEG);
    //     gcs().send_text(MAV_SEVERITY_INFO,"Yaw: %5.2f",(float)angles_to_target_rad.z*RAD_TO_DEG);
    //     _now = AP_HAL::millis();
    // }

    return true;
}

#endif // HAL_MOUNT_ENABLED
