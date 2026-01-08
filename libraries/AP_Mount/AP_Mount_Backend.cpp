#include "AP_Mount_Backend.h"
#if HAL_MOUNT_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Terrain/AP_Terrain.h>

extern const AP_HAL::HAL& hal;

//uint32_t _now = 0; // Uncomment this for debugging

#define AP_MOUNT_UPDATE_DT 0.02     // update rate in seconds.  update() should be called at this rate
#define AP_MOUNT_POI_REQUEST_TIMEOUT_MS 30000   // POI calculations continue to be updated for this many seconds after last request
#define AP_MOUNT_POI_RESULT_TIMEOUT_MS  3000    // POI calculations valid for 3 seconds
#define AP_MOUNT_POI_DIST_M_MAX         10000   // POI calculations limit of 10,000m (10km)

// Default init function for every mount
void AP_Mount_Backend::init()
{
    // setting default target sysid from parameters
    _target_sysid = _params.sysid_default.get();

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    // create a calculation thread for poi.
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Mount_Backend::calculate_poi, void),
                                      "mount_calc_poi",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Mount: failed to start POI thread");
    }
#endif
}

// set device id of this instance, for MNTx_DEVID parameter
void AP_Mount_Backend::set_dev_id(uint32_t id)
{
    _params.dev_id.set_and_save(int32_t(id));
}

// return true if this mount accepts roll targets
bool AP_Mount_Backend::has_roll_control() const
{
    return (_params.roll_angle_min < _params.roll_angle_max);
}

// return true if this mount accepts pitch targets
bool AP_Mount_Backend::has_pitch_control() const
{
    return (_params.pitch_angle_min < _params.pitch_angle_max);
}

bool AP_Mount_Backend::valid_mode(MAV_MOUNT_MODE mode) const
{
    switch (mode) {
    case MAV_MOUNT_MODE_RETRACT...MAV_MOUNT_MODE_HOME_LOCATION:
        return true;
    case MAV_MOUNT_MODE_ENUM_END:
        return false;
    }
    return false;
}

bool AP_Mount_Backend::set_mode(MAV_MOUNT_MODE mode)
{
    if (!valid_mode(mode)) {
        return false;
    }
    _mode = mode;
    return true;
}

// set angle target in degrees
// roll and pitch are in earth-frame
// yaw_is_earth_frame (aka yaw_lock) should be true if yaw angle is earth-frame, false if body-frame
void AP_Mount_Backend::set_angle_target(float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame)
{
    // enforce angle limits
    roll_deg = constrain_float(roll_deg, _params.roll_angle_min, _params.roll_angle_max);
    pitch_deg = constrain_float(pitch_deg, _params.pitch_angle_min, _params.pitch_angle_max);
    if (!yaw_is_earth_frame) {
        // only limit yaw if in body-frame.  earth-frame yaw limiting is backend specific
        // custom wrap code (instead of wrap_180) to better handle yaw of <= -180
        if (yaw_deg > 180) {
            yaw_deg -= 360;
        }
        yaw_deg = constrain_float(yaw_deg, _params.yaw_angle_min, _params.yaw_angle_max);
    }

    // set angle targets
    mnt_target.target_type = MountTargetType::ANGLE;
    mnt_target.angle_rad.roll = radians(roll_deg);
    mnt_target.angle_rad.pitch = radians(pitch_deg);
    mnt_target.angle_rad.yaw = radians(yaw_deg);
    mnt_target.angle_rad.yaw_is_ef = yaw_is_earth_frame;

    // set the mode to mavlink targeting
    set_mode(MAV_MOUNT_MODE_MAVLINK_TARGETING);

    // optionally set RC_TARGETING yaw lock state
    if (option_set(Options::RCTARGETING_LOCK_FROM_PREVMODE)) {
        set_yaw_lock(yaw_is_earth_frame);
    }
}

// sets rate target in deg/s
// yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
void AP_Mount_Backend::set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_is_earth_frame)
{
    // set rate targets
    mnt_target.target_type = MountTargetType::RATE;
    mnt_target.rate_rads.roll = radians(roll_degs);
    mnt_target.rate_rads.pitch = radians(pitch_degs);
    mnt_target.rate_rads.yaw = radians(yaw_degs);
    mnt_target.rate_rads.yaw_is_ef = yaw_is_earth_frame;

    // set the mode to mavlink targeting
    set_mode(MAV_MOUNT_MODE_MAVLINK_TARGETING);

    // optionally set RC_TARGETING yaw lock state
    if (option_set(Options::RCTARGETING_LOCK_FROM_PREVMODE)) {
        set_yaw_lock(yaw_is_earth_frame);
    }
}

// ARRC set fixed yaw angle after antenna alignment
void AP_Mount_Backend::set_fixed_yaw_angle(float fixed_yaw)
{
    // set angle targets
    _params.roll_stb_lead.set_and_save(fixed_yaw);
    // set the mode to mavlink targeting
    //_frontend.set_mode(_instance, MAV_MOUNT_MODE_GPS_POINT);
}

// ARRC set fixed yaw angle after antenna alignment
void AP_Mount_Backend::set_RotM_offset(Matrix3d rotm_off)
{
    // set Rotation matrix offset
    _params.rotM_offset = rotm_off;
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount_Backend::set_roi_target(const Location &target_loc)
{
    // set the target gps location
    _roi_target = target_loc;
    _roi_target_set = true;

    // set the mode to GPS tracking mode
    set_mode(MAV_MOUNT_MODE_GPS_POINT);

    // optionally set RC_TARGETING yaw lock state
    if (option_set(Options::RCTARGETING_LOCK_FROM_PREVMODE)) {
        set_yaw_lock(true);
    }
}

// clear_roi_target - clears target location that mount should attempt to point towards
void AP_Mount_Backend::clear_roi_target()
{
    // clear the target GPS location
    _roi_target_set = false;

    // reset the mode if in GPS tracking mode
    if (get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
        MAV_MOUNT_MODE default_mode = (MAV_MOUNT_MODE)_params.default_mode.get();
        set_mode(default_mode);
    }
}

// set_sys_target - sets system that mount should attempt to point towards
void AP_Mount_Backend::set_target_sysid(uint8_t sysid)
{
    _target_sysid = sysid;

    // set the mode to sysid tracking mode
    set_mode(MAV_MOUNT_MODE_SYSID_TARGET);

    // optionally set RC_TARGETING yaw lock state
    if (option_set(Options::RCTARGETING_LOCK_FROM_PREVMODE)) {
        set_yaw_lock(true);
    }
}

#if AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED
// process MOUNT_CONFIGURE messages received from GCS. deprecated.
void AP_Mount_Backend::handle_mount_configure(const mavlink_mount_configure_t &packet)
{
    set_mode((MAV_MOUNT_MODE)packet.mount_mode);
}
#endif

#if HAL_GCS_ENABLED
// send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
void AP_Mount_Backend::send_gimbal_device_attitude_status(mavlink_channel_t chan)
{
    if (suppress_heartbeat()) {
        // block heartbeat from transmitting to the GCS
        GCS_MAVLINK::disable_channel_routing(chan);
    }

    Quaternion att_quat;
    if (!get_attitude_quaternion(att_quat)) {
        return;
    }
    Vector3f ang_velocity { nanf(""), nanf(""), nanf("") };
    IGNORE_RETURN(get_angular_velocity(ang_velocity));

    // construct quaternion array
    const float quat_array[4] = {att_quat.q1, att_quat.q2, att_quat.q3, att_quat.q4};

    mavlink_msg_gimbal_device_attitude_status_send(chan,
                                                   0,   // target system
                                                   0,   // target component
                                                   AP_HAL::millis(),    // autopilot system time
                                                   get_gimbal_device_flags(),
                                                   quat_array,    // attitude expressed as quaternion
                                                   ang_velocity.x,    // roll axis angular velocity (NaN for unknown)
                                                   ang_velocity.y,    // pitch axis angular velocity (NaN for unknown)
                                                   ang_velocity.z,    // yaw axis angular velocity (NaN for unknown)
                                                   0,                                           // failure flags (not supported)
                                                   std::numeric_limits<double>::quiet_NaN(),    // delta_yaw (NaN for unknonw)
                                                   std::numeric_limits<double>::quiet_NaN(),    // delta_yaw_velocity (NaN for unknonw)
                                                   _instance + 1);  // gimbal_device_id
}
#endif

// return gimbal manager capability flags used by GIMBAL_MANAGER_INFORMATION message
uint32_t AP_Mount_Backend::get_gimbal_manager_capability_flags() const
{
    uint32_t cap_flags = GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT |
                         GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL |
                         GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS |
                         GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL |
                         GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL;

    // roll control
    if (has_roll_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK;
    }

    // pitch control
    if (has_pitch_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK;
    }

    // yaw control
    if (has_pan_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK;
    }

    return cap_flags;
}

// send a GIMBAL_MANAGER_INFORMATION message to GCS
void AP_Mount_Backend::send_gimbal_manager_information(mavlink_channel_t chan)
{
    mavlink_msg_gimbal_manager_information_send(chan,
                                                AP_HAL::millis(),                       // autopilot system time
                                                get_gimbal_manager_capability_flags(),  // bitmap of gimbal manager capability flags
                                                _instance + 1,                          // gimbal device id
                                                radians(_params.roll_angle_min),        // roll_min in radians
                                                radians(_params.roll_angle_max),        // roll_max in radians
                                                radians(_params.pitch_angle_min),       // pitch_min in radians
                                                radians(_params.pitch_angle_max),       // pitch_max in radians
                                                radians(_params.yaw_angle_min),         // yaw_min in radians
                                                radians(_params.yaw_angle_max));        // yaw_max in radians
}

// send a GIMBAL_MANAGER_STATUS message to GCS
void AP_Mount_Backend::send_gimbal_manager_status(mavlink_channel_t chan)
{
    uint32_t flags = GIMBAL_MANAGER_FLAGS_ROLL_LOCK | GIMBAL_MANAGER_FLAGS_PITCH_LOCK;

    if (_yaw_lock) {
        flags |= GIMBAL_MANAGER_FLAGS_YAW_LOCK;
    }

    mavlink_msg_gimbal_manager_status_send(chan,
                                           AP_HAL::millis(),    // autopilot system time
                                           flags,               // bitmap of gimbal manager flags
                                           _instance + 1,       // gimbal device id
                                           mavlink_control_id.sysid,    // primary control system id
                                           mavlink_control_id.compid,   // primary control component id
                                           0,                           // secondary control system id
                                           0);                          // secondary control component id
}

#if AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED
// process MOUNT_CONTROL messages received from GCS. deprecated.
void AP_Mount_Backend::handle_mount_control(const mavlink_mount_control_t &packet)
{
    switch (get_mode()) {
    case MAV_MOUNT_MODE_MAVLINK_TARGETING:
        // input_a : Pitch in centi-degrees (earth-frame)
        // input_b : Roll in centi-degrees (earth-frame)
        // input_c : Yaw in centi-degrees (interpreted as body-frame)
        set_angle_target(packet.input_b * 0.01, packet.input_a * 0.01, packet.input_c * 0.01, false);
        break;

    case MAV_MOUNT_MODE_GPS_POINT: {
        // input_a : lat in degE7
        // input_b : lon in degE7
        // input_c : alt  in cm (interpreted as above home)
        const Location target_location {
            packet.input_a,
            packet.input_b,
            packet.input_c,
            Location::AltFrame::ABOVE_HOME
        };
        set_roi_target(target_location);
        break;
    }

    case MAV_MOUNT_MODE_RETRACT:
    case MAV_MOUNT_MODE_NEUTRAL:
    case MAV_MOUNT_MODE_RC_TARGETING:
    case MAV_MOUNT_MODE_SYSID_TARGET:
    case MAV_MOUNT_MODE_HOME_LOCATION:
    default:
        // no effect in these modes
        break;
    }
}
#endif

// handle do_mount_control command.  Returns MAV_RESULT_ACCEPTED on success
MAV_RESULT AP_Mount_Backend::handle_command_do_mount_control(const mavlink_command_int_t &packet)
{
    const MAV_MOUNT_MODE new_mode = (MAV_MOUNT_MODE)packet.z;

    // interpret message fields based on mode
    switch (new_mode) {
    case MAV_MOUNT_MODE_RETRACT:
    case MAV_MOUNT_MODE_NEUTRAL:
    case MAV_MOUNT_MODE_RC_TARGETING:
    case MAV_MOUNT_MODE_HOME_LOCATION:
        // simply set mode
        set_mode(new_mode);
        return MAV_RESULT_ACCEPTED;

    case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
        // set target angles (in degrees) from mavlink message
        const float pitch_deg = packet.param1;  // param1: pitch (earth-frame, degrees)
        const float roll_deg = packet.param2;   // param2: roll (earth-frame, degrees)
        const float yaw_deg = packet.param3;    // param3: yaw (body-frame, degrees)

        // warn if angles are invalid to catch angles sent in centi-degrees
        if ((fabsf(pitch_deg) > 90) || (fabsf(roll_deg) > 180) || (fabsf(yaw_deg) > 360)) {
            send_warning_to_GCS("invalid angle targets");
            return MAV_RESULT_FAILED;
        }

        set_angle_target(packet.param2, packet.param1, packet.param3, false);
        return MAV_RESULT_ACCEPTED;
    }

    case MAV_MOUNT_MODE_GPS_POINT: {
        // set lat, lon, alt position targets from mavlink message

        // warn if lat, lon appear to be in param1,2 instead of param x,y as this indicates
        // sender is relying on a bug in AP-4.2's (and earlier) handling of MAV_CMD_DO_MOUNT_CONTROL
        if (!is_zero(packet.param1) && !is_zero(packet.param2) && packet.x == 0 && packet.y == 0) {
            send_warning_to_GCS("GPS_POINT target invalid");
            return MAV_RESULT_FAILED;
        }

        // param4: altitude in meters
        // x: latitude in degrees * 1E7
        // y: longitude in degrees * 1E7
        const Location target_location {
            packet.x,                       // latitude in degrees * 1E7
            packet.y,                       // longitude in degrees * 1E7
            (int32_t)packet.param4 * 100,   // alt converted from meters to cm
            Location::AltFrame::ABOVE_HOME
        };
        set_roi_target(target_location);
        return MAV_RESULT_ACCEPTED;
    }

    default:
        // invalid mode
        return MAV_RESULT_FAILED;
    }
}

// handle do_gimbal_manager_configure.  Returns MAV_RESULT_ACCEPTED on success
// requires original message in order to extract caller's sysid and compid
MAV_RESULT AP_Mount_Backend::handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // sanity check param1 and param2 values
    if ((packet.param1 < -3) || (packet.param1 > UINT8_MAX) || (packet.param2 < -3) || (packet.param2 > UINT8_MAX)) {
        return MAV_RESULT_FAILED;
    }

    // backup the current values so we can detect a change
    mavlink_control_id_t prev_control_id = mavlink_control_id;

    // convert negative packet1 and packet2 values
    int16_t new_sysid = packet.param1;
    switch (new_sysid) {
        case -1:
            // leave unchanged
            break;
        case -2:
            // set itself in control
            mavlink_control_id.sysid = msg.sysid;
            mavlink_control_id.compid = msg.compid;
            break;
        case -3:
            // remove control if currently in control
            if ((mavlink_control_id.sysid == msg.sysid) && (mavlink_control_id.compid == msg.compid)) {
                mavlink_control_id.sysid = 0;
                mavlink_control_id.compid = 0;
            }
            break;
        default:
            mavlink_control_id.sysid = packet.param1;
            mavlink_control_id.compid = packet.param2;
            break;
    }

    // send gimbal_manager_status if control has changed
    if (prev_control_id != mavlink_control_id) {
        GCS_SEND_MESSAGE(MSG_GIMBAL_MANAGER_STATUS);
    }

    return MAV_RESULT_ACCEPTED;
}

// handle a GLOBAL_POSITION_INT message
bool AP_Mount_Backend::handle_global_position_int(uint8_t msg_sysid, const mavlink_global_position_int_t &packet)
{
    if (_target_sysid != msg_sysid) {
        return false;
    }

    _target_sysid_location.lat = packet.lat;
    _target_sysid_location.lng = packet.lon;
    // global_position_int.alt is *UP*, so is location.
    _target_sysid_location.set_alt_cm(packet.alt*0.1, Location::AltFrame::ABSOLUTE);
    _target_sysid_location_set = true;

    return true;
}

#if HAL_LOGGING_ENABLED
// write mount log packet
void AP_Mount_Backend::write_log(uint64_t timestamp_us)
{
    // return immediately if no yaw estimate
    float ahrs_yaw = AP::ahrs().get_yaw();
    if (isnan(ahrs_yaw)) {
        return;
    }

    const auto nanf = AP::logger().quiet_nanf();

    // get_attitude_quaternion and convert to Euler angles
    float roll = nanf;
    float pitch = nanf;
    float yaw_bf = nanf;
    float yaw_ef = nanf;
    if (_frontend.get_attitude_euler(_instance, roll, pitch, yaw_bf)) {
        yaw_ef = wrap_180(yaw_bf + degrees(ahrs_yaw));
    }

    // get mount's target (desired) angles and convert yaw to earth frame
    float target_roll = nanf;
    float target_pitch = nanf;
    float target_yaw = nanf;
    bool target_yaw_is_ef = false;
    IGNORE_RETURN(get_angle_target(target_roll, target_pitch, target_yaw, target_yaw_is_ef));

    // get rangefinder distance
    float rangefinder_dist = nanf;
    IGNORE_RETURN(get_rangefinder_distance(rangefinder_dist));

    const struct log_Mount pkt {
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(LOG_MOUNT_MSG)),
        time_us       : (timestamp_us > 0) ? timestamp_us : AP_HAL::micros64(),
        instance      : _instance,
        desired_roll  : target_roll,
        actual_roll   : roll,
        desired_pitch : target_pitch,
        actual_pitch  : pitch,
        desired_yaw_bf: target_yaw_is_ef ? nanf : target_yaw,
        actual_yaw_bf : yaw_bf,
        desired_yaw_ef: target_yaw_is_ef ? target_yaw : nanf,
        actual_yaw_ef : yaw_ef,
        rangefinder_dist : rangefinder_dist,
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
}
#endif

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
// get poi information.  Returns true on success and fills in gimbal attitude, location and poi location
bool AP_Mount_Backend::get_poi(uint8_t instance, Quaternion &quat, Location &loc, Location &poi_loc)
{
    WITH_SEMAPHORE(poi_calculation.sem);

    // record time of request
    const uint32_t now_ms = AP_HAL::millis();
    poi_calculation.poi_request_ms = now_ms;

    // check if poi calculated recently
    if (now_ms - poi_calculation.poi_update_ms > AP_MOUNT_POI_RESULT_TIMEOUT_MS) {
        return false;
    }

    // check attitude is valid
    if (poi_calculation.att_quat.is_nan()) {
        return false;
    }

    quat = poi_calculation.att_quat;
    loc = poi_calculation.loc;
    poi_loc = poi_calculation.poi_loc;
    return true;
}

// calculate the Location that the gimbal is pointing at
void AP_Mount_Backend::calculate_poi()
{
    while (true) {
        // run this loop at 10hz
        hal.scheduler->delay(100);

        // calculate poi if requested within last 30 seconds
        {
            WITH_SEMAPHORE(poi_calculation.sem);
            if ((poi_calculation.poi_request_ms == 0) ||
                (AP_HAL::millis() - poi_calculation.poi_request_ms > AP_MOUNT_POI_REQUEST_TIMEOUT_MS)) {
                continue;
            }
        }

        // get the current location of vehicle
        const AP_AHRS &ahrs = AP::ahrs();
        Location curr_loc;
        if (!ahrs.get_location(curr_loc)) {
            continue;
        }

        // change vehicle alt to AMSL
        curr_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);

        // project forward from vehicle looking for terrain
        // start testing at vehicle's location
        Location test_loc = curr_loc;
        Location prev_test_loc = curr_loc;

        // get terrain altitude (AMSL) at test_loc
        auto terrain = AP_Terrain::get_singleton();
        float terrain_amsl_m;
        if ((terrain == nullptr) || !terrain->height_amsl(test_loc, terrain_amsl_m, true)) {
            continue;
        }

        // retrieve gimbal attitude
        Quaternion quat;
        if (!get_attitude_quaternion(quat)) {
            // gimbal attitude unavailable
            continue;
        }

        // iteratively move test_loc forward until its alt-above-sea-level is below terrain-alt-above-sea-level
        const float dist_increment_m = MAX(terrain->get_grid_spacing(), 10);
        const float mount_pitch_deg = degrees(quat.get_euler_pitch());
        const float mount_yaw_ef_deg = wrap_180(degrees(quat.get_euler_yaw()) + degrees(ahrs.get_yaw()));
        float total_dist_m = 0;
        bool get_terrain_alt_success = true;
        float prev_terrain_amsl_m = terrain_amsl_m;
        while (total_dist_m < AP_MOUNT_POI_DIST_M_MAX && (test_loc.alt * 0.01) > terrain_amsl_m) {
            total_dist_m += dist_increment_m;

            // backup previous test location and terrain amsl
            prev_test_loc = test_loc;
            prev_terrain_amsl_m = terrain_amsl_m;

            // move test location forward
            test_loc.offset_bearing_and_pitch(mount_yaw_ef_deg, mount_pitch_deg, dist_increment_m);

            // get terrain's alt-above-sea-level (at test_loc)
            // fail if terrain alt cannot be retrieved
            if (!terrain->height_amsl(test_loc, terrain_amsl_m, true) || std::isnan(terrain_amsl_m)) {
                get_terrain_alt_success = false;
                continue;
            }
        }

        // if a fail occurred above when getting terrain alt then restart calculations from the beginning
        if (!get_terrain_alt_success) {
            continue;
        }

        if (total_dist_m >= AP_MOUNT_POI_DIST_M_MAX) {
            // unable to find terrain within dist_max
            continue;
        }

        // test location has dropped below terrain
        // interpolate along line between prev_test_loc and test_loc
        float dist_interp_m = linear_interpolate(0, dist_increment_m, 0, prev_test_loc.alt * 0.01 - prev_terrain_amsl_m, test_loc.alt * 0.01 - terrain_amsl_m);
        {
            WITH_SEMAPHORE(poi_calculation.sem);
            poi_calculation.poi_loc = prev_test_loc;
            poi_calculation.poi_loc.offset_bearing_and_pitch(mount_yaw_ef_deg, mount_pitch_deg, dist_interp_m);
            poi_calculation.att_quat = {quat[0], quat[1], quat[2], quat[3]};
            poi_calculation.loc = curr_loc;
            poi_calculation.poi_update_ms = AP_HAL::millis();
        }
    }
}
#endif

// change to RC_TARGETING mode if rc inputs have changed by more than the dead zone
// should be called on every update
void AP_Mount_Backend::set_rctargeting_on_rcinput_change()
{
    // exit immediately if no RC input
    if (!rc().has_valid_input()) {
        return;
    }

    const RC_Channel *roll_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_ROLL : RC_Channel::AUX_FUNC::MOUNT2_ROLL);
    const RC_Channel *pitch_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_PITCH : RC_Channel::AUX_FUNC::MOUNT2_PITCH);
    const RC_Channel *yaw_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_YAW : RC_Channel::AUX_FUNC::MOUNT2_YAW);

    // get rc input
    const int16_t roll_in = (roll_ch == nullptr) ? 0 : roll_ch->get_radio_in();
    const int16_t pitch_in = (pitch_ch == nullptr) ? 0 : pitch_ch->get_radio_in();
    const int16_t yaw_in = (yaw_ch == nullptr) ? 0 : yaw_ch->get_radio_in();

    if (!last_rc_input.initialised) {
            // The first time through, initial RC inputs should be set, but not used
            last_rc_input.initialised = true;
            last_rc_input.roll_in = roll_in;
            last_rc_input.pitch_in = pitch_in;
            last_rc_input.yaw_in = yaw_in;
    }
    // if not in RC_TARGETING or RETRACT modes then check for RC change
    if (get_mode() != MAV_MOUNT_MODE_RC_TARGETING && get_mode() != MAV_MOUNT_MODE_RETRACT) {
        // get dead zones
        const int16_t roll_dz = (roll_ch == nullptr) ? 10 : MAX(roll_ch->get_dead_zone(), 10);
        const int16_t pitch_dz = (pitch_ch == nullptr) ? 10 : MAX(pitch_ch->get_dead_zone(), 10);
        const int16_t yaw_dz = (yaw_ch == nullptr) ? 10 : MAX(yaw_ch->get_dead_zone(), 10);

        // check if RC input has changed by more than the dead zone
        if ((abs(last_rc_input.roll_in - roll_in) > roll_dz) ||
            (abs(last_rc_input.pitch_in - pitch_in) > pitch_dz) ||
            (abs(last_rc_input.yaw_in - yaw_in) > yaw_dz)) {
                set_mode(MAV_MOUNT_MODE_RC_TARGETING);
        }
    }

    // if NOW in RC_TARGETING or RETRACT mode then store last RC input (mode might have changed)
    if (get_mode() == MAV_MOUNT_MODE_RC_TARGETING || get_mode() == MAV_MOUNT_MODE_RETRACT) {
        last_rc_input.roll_in = roll_in;
        last_rc_input.pitch_in = pitch_in;
        last_rc_input.yaw_in = yaw_in;
    }
}

// get pilot input (in the range -1 to +1) received through RC
void AP_Mount_Backend::get_rc_input(float& roll_in, float& pitch_in, float& yaw_in) const
{
    const RC_Channel *roll_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_ROLL : RC_Channel::AUX_FUNC::MOUNT2_ROLL);
    const RC_Channel *pitch_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_PITCH : RC_Channel::AUX_FUNC::MOUNT2_PITCH);
    const RC_Channel *yaw_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_YAW : RC_Channel::AUX_FUNC::MOUNT2_YAW);

    roll_in = 0;
    if ((roll_ch != nullptr) && (roll_ch->get_radio_in() > 0)) {
        roll_in = roll_ch->norm_input_dz();
    }

    pitch_in = 0;
    if ((pitch_ch != nullptr) && (pitch_ch->get_radio_in() > 0)) {
        pitch_in = pitch_ch->norm_input_dz();
    }

    yaw_in = 0;
    if ((yaw_ch != nullptr) && (yaw_ch->get_radio_in() > 0)) {
        yaw_in = yaw_ch->norm_input_dz();
    }
}

// get angle or rate targets from pilot RC
// target_type will be either ANGLE or RATE, rpy will be the target angle in deg or rate in deg/s
void AP_Mount_Backend::get_rc_target(MountTargetType& target_type, MountTarget& target_rpy) const
{
    // get RC input from pilot
    float roll_in, pitch_in, yaw_in;
    get_rc_input(roll_in, pitch_in, yaw_in);

    // yaw frame
    target_rpy.yaw_is_ef = _yaw_lock;

    // if RC_RATE is zero, targets are angle
    if (_params.rc_rate_max <= 0) {
        target_type = MountTargetType::ANGLE;

        // roll angle
        target_rpy.roll = radians(((roll_in + 1.0f) * 0.5f * (_params.roll_angle_max - _params.roll_angle_min) + _params.roll_angle_min));

        // pitch angle
        target_rpy.pitch = radians(((pitch_in + 1.0f) * 0.5f * (_params.pitch_angle_max - _params.pitch_angle_min) + _params.pitch_angle_min));

        // yaw angle
        if (target_rpy.yaw_is_ef) {
            // if yaw is earth-frame pilot yaw input control angle from -180 to +180 deg
            target_rpy.yaw = yaw_in * M_PI;
        } else {
            // yaw target in body frame so apply body frame limits
            target_rpy.yaw = radians(((yaw_in + 1.0f) * 0.5f * (_params.yaw_angle_max - _params.yaw_angle_min) + _params.yaw_angle_min));
        }
        return;
    }

    // calculate rate targets
    target_type = MountTargetType::RATE;
    const float rc_rate_max_rads = radians(_params.rc_rate_max.get());
    target_rpy.roll = roll_in * rc_rate_max_rads;
    target_rpy.pitch = pitch_in * rc_rate_max_rads;
    target_rpy.yaw = yaw_in * rc_rate_max_rads;
}

// get angle targets (in radians) to a Location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_location(const Location &target, MountTarget& angle_rad) const
{
    // Named constants for clarity and maintainability
    constexpr double LAT_LON_TO_RAD = 1.0e-7 * DEG_TO_RAD;  // Convert 1e7 scaled degrees to radians
    constexpr float LOCATION_SCALING_FACTOR = 0.01113195f;   // meters per 1e-7 degree at equator
    constexpr float MIN_DISTANCE_M = 10.0f;                  // Minimum distance threshold in meters
    constexpr float SLOPE_THRESHOLD = 0.35f;                 // Slope threshold for mode selection
    constexpr float ELEVATION_THRESHOLD_RAD = 70.0f * DEG_TO_RAD;  // 70 degrees in radians (~1.22 rad)
    constexpr float PITCH_LIMIT_RAD = 1.31f;                 // ~75 degrees pitch limit
    constexpr float ROLL_LIMIT_RAD = 0.6f;                   // ~34 degrees roll limit
    constexpr float CM_TO_M = 0.01f;                         // Centimeters to meters

    // ============================================================
    // EARLY EXIT CHECKS - Perform before any expensive calculations
    // ============================================================

    // Exit immediately if vehicle's location is unavailable
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    // Exit immediately if target location is invalid
    if (!target.initialised()) {
        return false;
    }

    // Get altitudes early - exit if unavailable (before expensive trig calculations)
    int32_t target_alt_cm = 0;
    if (!target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm)) {
        return false;
    }
    int32_t current_alt_cm = 0;
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt_cm)) {
        return false;
    }

    // ============================================================
    // COORDINATE CONVERSION - Cache trigonometric values
    // ============================================================

    // Convert lat/lon to radians (compute once)
    const double curr_lat_rad = current_loc.lat * LAT_LON_TO_RAD;
    const double tar_lat_rad = target.lat * LAT_LON_TO_RAD;
    const double delta_lat = tar_lat_rad - curr_lat_rad;
    const double delta_lng = Location::diff_longitude(target.lng, current_loc.lng) * LAT_LON_TO_RAD;

    // Cache trigonometric values (expensive operations - compute once, use multiple times)
    const double sin_curr_lat = sin(curr_lat_rad);
    const double cos_curr_lat = cos(curr_lat_rad);
    const double sin_tar_lat = sin(tar_lat_rad);
    const double cos_tar_lat = cos(tar_lat_rad);
    const double sin_delta_lng = sin(delta_lng);
    const double cos_delta_lng = cos(delta_lng);
    const double sin_half_delta_lat = sin(delta_lat * 0.5);
    const double sin_half_delta_lng = sin(delta_lng * 0.5);

    // ============================================================
    // HAVERSINE FORMULA - Compute horizontal distance to target
    // ============================================================

    const double haversine_a = sin_half_delta_lat * sin_half_delta_lat +
                               cos_curr_lat * cos_tar_lat * sin_half_delta_lng * sin_half_delta_lng;
    const double target_distance = 2.0 * RADIUS_OF_EARTH * atan2(sqrt(haversine_a), sqrt(1.0 - haversine_a));

    // ============================================================
    // BEARING CALCULATION - Using cached trig values
    // ============================================================

    const double bearing_y = sin_delta_lng * cos_tar_lat;
    const double bearing_x = cos_curr_lat * sin_tar_lat - sin_curr_lat * cos_tar_lat * cos_delta_lng;
    const double bearing = atan2(bearing_y, bearing_x);

    // Cache bearing trig values (used in target vector calculation)
    const double sin_bearing = sin(bearing);
    const double cos_bearing = cos(bearing);

    // Fixed yaw from parameters (cache trig values)
    const double fixed_yaw = _params.roll_stb_lead * DEG_TO_RAD;
    const double sin_fixed_yaw = sin(fixed_yaw);
    const double cos_fixed_yaw = cos(fixed_yaw);

    // ============================================================
    // TARGET VECTOR - Components in target's reference frame (NWU)
    // ============================================================

    // Using cached sin/cos values instead of recomputing
    const double y = target_distance * (sin_bearing * cos_fixed_yaw - cos_bearing * sin_fixed_yaw);
    const double x = -target_distance * (sin_bearing * sin_fixed_yaw + cos_bearing * cos_fixed_yaw);

    // Compute height difference in meters (NWU convention)
    const double z = (current_alt_cm - target_alt_cm) * CM_TO_M + _params.ARRC_z_offset;

    // Compute slope (avoid division by zero)
    // Note: Using target_distance instead of redundant get_distance() call
    const float slope = (target_distance > 0.001) ? fabsf(static_cast<float>(z / target_distance)) : 0.0f;

    // Get AUT elevation in radians
    const double el = _params.ARRC_elev * DEG_TO_RAD;

    // Cache elevation trig values (used in Vpol mode)
    const double sin_el = sin(el);
    const double cos_el = cos(el);

    // ============================================================
    // GIMBAL ANGLE CALCULATION - Based on mode selection
    // ============================================================

    // Alexmos gimbal convention: Pitch down (+), Roll right (+), Yaw right (+)
    // Gremsy gimbal convention: Pitch down (?), Roll right (?), Yaw right (?)
    // This technique's convention: Pitch down (-), Roll right (-), Yaw right (+)
    // Position (x,y,z) is NWU convention with the AUT as the origin

    // Compute 3D distance for mode 2 and 3 (precompute before branches if needed)
    const double x_sq = x * x;
    const double y_sq = y * y;
    const double z_sq = z * z;
    const double D_sq = x_sq + y_sq + z_sq;
    const double D = sqrt(D_sq);

    if (D > MIN_DISTANCE_M) {
        // Mode 0: Original ArduPilot mode
        // Condition: pitch_stb_lead == 0 OR (low slope AND high elevation)
        if (is_zero(_params.pitch_stb_lead) || (slope < SLOPE_THRESHOLD && el > ELEVATION_THRESHOLD_RAD)) {

            // Calculate GPS vectors for ArduPilot standard method
            const float avg_lat_rad = (current_loc.lat + target.lat) * 0.5e-7f * DEG_TO_RAD;
            const float GPS_vector_x = Location::diff_longitude(target.lng, current_loc.lng) * cosf(avg_lat_rad) * LOCATION_SCALING_FACTOR;
            const float GPS_vector_y = (target.lat - current_loc.lat) * LOCATION_SCALING_FACTOR;
            const float GPS_vector_z = (target_alt_cm - current_alt_cm) * CM_TO_M;

            // Calculate roll, pitch, yaw angles
            angle_rad.roll = 0.0f;
            angle_rad.pitch = atan2f(GPS_vector_z, static_cast<float>(target_distance));
            angle_rad.yaw = atan2f(GPS_vector_x, GPS_vector_y);
            angle_rad.yaw_is_ef = true;
        }
        // Mode 1: Probe plane parallel to AUT plane
        else if (is_zero(_params.pitch_stb_lead - 1.0f)) {

            angle_rad.pitch = static_cast<float>(-el);
            angle_rad.roll = 0.0f;
            // Use wrap_PI directly on radians instead of deg->wrap->rad conversion
            angle_rad.yaw = wrap_PI(static_cast<float>(fixed_yaw));
            angle_rad.yaw_is_ef = true;
        }
        // Mode 2: Hpol aligned mode
        else if (is_zero(_params.pitch_stb_lead - 2.0f)) {

            const double A = sqrt(x_sq + z_sq);

            // Avoid division by zero
            if (A < 0.001 || D < 0.001) {
                return false;
            }

            const double inv_D = 1.0 / D;
            const double inv_A = 1.0 / A;
            const double inv_AD = inv_A * inv_D;

            Matrix3d RotM(-x * inv_D,          -y * inv_D,       -z * inv_D,
                          -x * y * inv_AD,      A * inv_D,       -y * z * inv_AD,
                           z * inv_A,           0.0,             -x * inv_A);

            RotM = _params.rotM_offset * RotM;

            // tilt calcs = atan2(Reb(1,3), Reb(3,3))
            angle_rad.pitch = static_cast<float>(atan2(RotM.a.z, RotM.c.z));

            // roll calcs = atan2(-Reb(2,3), sqrt(1-Reb(2,3)^2))
            const double bz_sq = RotM.b.z * RotM.b.z;
            angle_rad.roll = static_cast<float>(-atan2(-RotM.b.z, sqrt(1.0 - bz_sq)));

            // pan calcs = atan2(Reb(2,1), Reb(2,2))
            angle_rad.yaw = static_cast<float>(atan2(RotM.b.x, RotM.b.y) + fixed_yaw);
            angle_rad.yaw_is_ef = true;
        }
        // Mode 3: Vpol aligned mode
        else if (is_zero(_params.pitch_stb_lead - 3.0f)) {

            // Using cached sin_el, cos_el values
            const double cos_el_sq = cos_el * cos_el;
            const double sin_2el = 2.0 * sin_el * cos_el;  // sin(2*el) = 2*sin(el)*cos(el)
            const double cos_2el = cos_el_sq - sin_el * sin_el;  // cos(2*el) = cos^2(el) - sin^2(el)

            const double A_sq = (x_sq - z_sq) * cos_el_sq + y_sq + z_sq - x * z * sin_2el;
            const double A = sqrt(A_sq);
            const double B = sqrt((x_sq - z_sq) * cos_2el + x_sq + 2.0 * y_sq + z_sq - 2.0 * x * z * sin_2el);

            // Avoid division by zero
            if (A < 0.001 || B < 0.001 || D < 0.001) {
                return false;
            }

            const double inv_D = 1.0 / D;
            const double inv_A = 1.0 / A;
            const double inv_DB = inv_D / B;

            Matrix3d RotM(-x * inv_D,
                          -y * inv_D,
                          -z * inv_D,
                           y * cos_el * inv_A,
                          -(x * cos_el - z * sin_el) * inv_A,
                          -y * sin_el * inv_A,
                           (M_SQRT2 * ((y_sq + z_sq) * sin_el - x * z * cos_el)) * inv_DB,
                          -(M_SQRT2 * y * (z * cos_el + x * sin_el)) * inv_DB,
                           M_SQRT2 * ((x_sq + y_sq) * cos_el - x * z * sin_el) * B / (2.0 * D * A_sq));

            RotM = _params.rotM_offset * RotM;

            // tilt calcs = atan2(Reb(1,3), Reb(3,3))
            angle_rad.pitch = static_cast<float>(atan2(RotM.a.z, RotM.c.z));

            // roll calcs = atan2(-Reb(2,3), sqrt(1-Reb(2,3)^2))
            const double bz_sq = RotM.b.z * RotM.b.z;
            angle_rad.roll = static_cast<float>(-atan2(-RotM.b.z, sqrt(1.0 - bz_sq)));

            // pan calcs = atan2(Reb(2,1), Reb(2,2))
            angle_rad.yaw = static_cast<float>(atan2(RotM.b.x, RotM.b.y) + fixed_yaw);
            angle_rad.yaw_is_ef = true;
        }
    }

    // Apply angle constraints
    angle_rad.pitch = constrain_float(angle_rad.pitch, -PITCH_LIMIT_RAD, PITCH_LIMIT_RAD);
    angle_rad.roll = constrain_float(angle_rad.roll, -ROLL_LIMIT_RAD, ROLL_LIMIT_RAD);

    // For debugging: Uncomment to verify math during development
    // if (AP_HAL::millis() - _now > 3000) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "Target Dist: %5.2f", static_cast<float>(target_distance));
    //     gcs().send_text(MAV_SEVERITY_INFO, "Bearing: %5.2f", static_cast<float>(bearing * RAD_TO_DEG));
    //     gcs().send_text(MAV_SEVERITY_INFO, "Target X: %5.2f", static_cast<float>(x));
    //     gcs().send_text(MAV_SEVERITY_INFO, "Target Y: %5.2f", static_cast<float>(y));
    //     gcs().send_text(MAV_SEVERITY_INFO, "Target Z: %5.2f", static_cast<float>(z));
    //     gcs().send_text(MAV_SEVERITY_INFO, "Pitch: %5.2f", angle_rad.pitch * RAD_TO_DEG);
    //     gcs().send_text(MAV_SEVERITY_INFO, "Roll: %5.2f", angle_rad.roll * RAD_TO_DEG);
    //     gcs().send_text(MAV_SEVERITY_INFO, "Yaw: %5.2f", angle_rad.yaw * RAD_TO_DEG);
    //     _now = AP_HAL::millis();  // Don't forget to uncomment declaration at the very top
    // }

    return true;
}

// get angle targets (in radians) to ROI location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_roi(MountTarget& angle_rad) const
{
    if (!_roi_target_set) {
        return false;
    }
    return get_angle_target_to_location(_roi_target, angle_rad);
}

// return body-frame yaw angle from a mount target
float AP_Mount_Backend::MountTarget::get_bf_yaw() const
{
    if (yaw_is_ef) {
        // convert to body-frame
        return wrap_PI(yaw - AP::ahrs().get_yaw());
    }

    // target is already body-frame
    return yaw;
}

// return earth-frame yaw angle from a mount target
float AP_Mount_Backend::MountTarget::get_ef_yaw() const
{
    if (yaw_is_ef) {
        // target is already earth-frame
        return yaw;
    }

    // convert to earth-frame
    return wrap_PI(yaw + AP::ahrs().get_yaw());
}

// sets roll, pitch, yaw and yaw_is_ef
void AP_Mount_Backend::MountTarget::set(const Vector3f& rpy, bool yaw_is_ef_in)
{
    roll  = rpy.x;
    pitch = rpy.y;
    yaw   = rpy.z;
    yaw_is_ef = yaw_is_ef_in;
}

// update angle targets using a given rate target
// the resulting angle_rad yaw frame will match the rate_rad yaw frame
// assumes a 50hz update rate
void AP_Mount_Backend::update_angle_target_from_rate(const MountTarget& rate_rad, MountTarget& angle_rad) const
{
    // update roll and pitch angles and apply limits
    angle_rad.roll = constrain_float(angle_rad.roll + rate_rad.roll * AP_MOUNT_UPDATE_DT, radians(_params.roll_angle_min), radians(_params.roll_angle_max));
    angle_rad.pitch = constrain_float(angle_rad.pitch + rate_rad.pitch * AP_MOUNT_UPDATE_DT, radians(_params.pitch_angle_min), radians(_params.pitch_angle_max));

    // ensure angle yaw frames matches rate yaw frame
    if (angle_rad.yaw_is_ef != rate_rad.yaw_is_ef) {
        if (rate_rad.yaw_is_ef) {
            angle_rad.yaw = angle_rad.get_ef_yaw();
        } else {
            angle_rad.yaw = angle_rad.get_bf_yaw();
        }
        angle_rad.yaw_is_ef = rate_rad.yaw_is_ef;
    }

    // update yaw angle target
    angle_rad.yaw = angle_rad.yaw + rate_rad.yaw * AP_MOUNT_UPDATE_DT;
    if (angle_rad.yaw_is_ef) {
        // if earth-frame yaw wraps between += 180 degrees
        angle_rad.yaw = wrap_PI(angle_rad.yaw);
    } else {
        // if body-frame constrain yaw to body-frame limits
        angle_rad.yaw = constrain_float(angle_rad.yaw, radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));
    }
}

// helper function to provide GIMBAL_DEVICE_FLAGS for use in GIMBAL_DEVICE_ATTITUDE_STATUS message
uint16_t AP_Mount_Backend::get_gimbal_device_flags() const
{
    // get yaw lock state by mode
    bool yaw_lock_state = false;
    switch (_mode) {
    case MAV_MOUNT_MODE_RETRACT:
    case MAV_MOUNT_MODE_NEUTRAL:
        // these modes always use body-frame yaw (aka follow)
        yaw_lock_state = false;
        break;
    case MAV_MOUNT_MODE_MAVLINK_TARGETING:
        switch (mnt_target.target_type) {
        case MountTargetType::RATE:
            yaw_lock_state = mnt_target.rate_rads.yaw_is_ef;
            break;
        case MountTargetType::ANGLE:
            yaw_lock_state = mnt_target.angle_rad.yaw_is_ef;
            break;
        }
        break;
    case MAV_MOUNT_MODE_RC_TARGETING:
        yaw_lock_state = _yaw_lock;
        break;
    case MAV_MOUNT_MODE_GPS_POINT:
    case MAV_MOUNT_MODE_SYSID_TARGET:
    case MAV_MOUNT_MODE_HOME_LOCATION:
        // these modes always use earth-frame yaw (aka lock)
        yaw_lock_state = true;
        break;
    case MAV_MOUNT_MODE_ENUM_END:
        // unsupported
        yaw_lock_state = false;
        break;
    }

    const uint16_t flags = (get_mode() == MAV_MOUNT_MODE_RETRACT ? GIMBAL_DEVICE_FLAGS_RETRACT : 0) |
                           (get_mode() == MAV_MOUNT_MODE_NEUTRAL ? GIMBAL_DEVICE_FLAGS_NEUTRAL : 0) |
                           GIMBAL_DEVICE_FLAGS_ROLL_LOCK | // roll angle is always earth-frame
                           GIMBAL_DEVICE_FLAGS_PITCH_LOCK| // pitch angle is always earth-frame, yaw_angle is always body-frame
                           GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME | // yaw angle is always in vehicle-frame
                           (yaw_lock_state ? GIMBAL_DEVICE_FLAGS_YAW_LOCK : 0);
    return flags;
}

// get angle targets (in radians) to home location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_home(MountTarget& angle_rad) const
{
    // exit immediately if home is not set
    if (!AP::ahrs().home_is_set()) {
        return false;
    }
    return get_angle_target_to_location(AP::ahrs().get_home(), angle_rad);
}

// get angle targets (in radians) to a vehicle with sysid of  _target_sysid
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_sysid(MountTarget& angle_rad) const
{
    // exit immediately if sysid is not set or no location available
    if (!_target_sysid_location_set) {
        return false;
    }
    if (!_target_sysid) {
        return false;
    }
    return get_angle_target_to_location(_target_sysid_location, angle_rad);
}

// get target rate in deg/sec. returns true on success
bool AP_Mount_Backend::get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame)
{
    if (mnt_target.target_type == MountTargetType::RATE) {
        roll_degs = degrees(mnt_target.rate_rads.roll);
        pitch_degs = degrees(mnt_target.rate_rads.pitch);
        yaw_degs = degrees(mnt_target.rate_rads.yaw);
        yaw_is_earth_frame = mnt_target.rate_rads.yaw_is_ef;
        return true;
    }
    return false;
}

// get target angle in deg. returns true on success
bool AP_Mount_Backend::get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame)
{
    if (mnt_target.target_type == MountTargetType::ANGLE) {
        roll_deg = degrees(mnt_target.angle_rad.roll);
        pitch_deg = degrees(mnt_target.angle_rad.pitch);
        yaw_deg = degrees(mnt_target.angle_rad.yaw);
        yaw_is_earth_frame = mnt_target.angle_rad.yaw_is_ef;
        return true;
    }
    return false;
}

// sent warning to GCS.  Warnings are throttled to at most once every 30 seconds
void AP_Mount_Backend::send_warning_to_GCS(const char* warning_str)
{
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_warning_ms < 30000) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Mount: %s", warning_str);
    _last_warning_ms = now_ms;
}

#endif // HAL_MOUNT_ENABLED
