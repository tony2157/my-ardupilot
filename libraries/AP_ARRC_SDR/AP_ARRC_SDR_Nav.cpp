/*
 * AP_ARRC_SDR sensor sample streams to the companion computer (ARRC_NAV_*, ARRC_GIMBAL_POINTING).
 *
 * The companion records them in its HDF5 data next to the SDR data, so the data can be processed without
 * the autopilot log. Each stream runs at the rate the autopilot logs the same data: IMU, MAG, BARO and ARGM
 * samples are sent from the vehicle tasks that log them (the IMU follows LOG_BITMASK IMU / IMU_FAST), GPS
 * and ECEF once per new fix. That cadence holds whether or not the log is being written. The companion
 * requests each message with MAV_CMD_SET_MESSAGE_INTERVAL on its link: that subscribes the link and caps
 * the stream at the requested rate (IMU_FAST at 400 Hz or a slow link must not starve the heartbeat, the
 * ACKs and the GCS downlink); a sample is also held back when the channel's tx buffer is nearly full.
 * The cap is the requested interval itself, not the stream scheduler's timing, which slows down while
 * parameters or a mission are transferred. Instance 0 of each sensor; samples carry the time they were taken
 * on the boot clock. The IMU samples also carry the INS delta angle and velocity summed since the channel's
 * previous sample (unfiltered, as the EKF uses them), the GPS samples the driver's corrected fix time in
 * microseconds and the gimbal samples the gimbal's own attitude report with its receive time.
 */
#include "AP_ARRC_SDR.h"

#if AP_ARRC_SDR_ENABLED

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Mount/AP_Mount.h>
#include <GCS_MAVLink/GCS.h>

// tx space kept free on a channel when a sample is sent: a heartbeat, an ACK and an ARRC_SENSOR_RAW
static constexpr uint16_t NAV_TX_HEADROOM_BYTES = 200;

// a channel's IMU deltas are summed while its last ARRC_NAV_IMU is this recent; after a longer pause the
// next sample starts a new interval (delta_time 0)
static constexpr uint32_t NAV_IMU_DELTA_IDLE_MS = 2000;

// the driver's corrected fix time is used when it is at most this much older than the fix's receive time;
// an older one belongs to an earlier fix (5 or 10 Hz)
static constexpr uint32_t NAV_GPS_EPOCH_MAX_AGE_US = 80000;

// the message of each stream
static constexpr ap_message nav_msg[uint8_t(AP_ARRC_SDR::NavStream::COUNT)] {
    MSG_ARRC_NAV_IMU, MSG_ARRC_NAV_MAG, MSG_ARRC_NAV_BARO, MSG_ARRC_NAV_GPS, MSG_ARRC_NAV_ECEF,
    MSG_ARRC_GIMBAL_POINTING,
};

// boot time of a sample stamped with the 32-bit microsecond clock (less than 71 minutes old)
static uint64_t sample_time_us(uint64_t now_us, uint32_t sample_us)
{
    return now_us - uint32_t(uint32_t(now_us) - sample_us);
}

// boot time of a sample stamped with the 32-bit millisecond clock
static uint64_t sample_time_ms_to_us(uint64_t now_us, uint32_t sample_ms)
{
    const uint64_t now_ms = now_us / 1000U;
    return (now_ms - uint32_t(uint32_t(now_ms) - sample_ms)) * 1000U;
}

void AP_ARRC_SDR::nav_due(NavStream stream, mavlink_channel_t chan)
{
    if (chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return;
    }
    // a channel with an interval gets the samples from the vehicle tasks; a single request gets the latest one
    uint16_t interval_ms;
    if (!chan_interval(chan, nav_msg[uint8_t(stream)], interval_ms)) {
        nav_send(stream, chan);
    }
}

bool AP_ARRC_SDR::nav_ready(NavStream stream, uint8_t chan, uint16_t payload_bytes)
{
    const uint8_t s = uint8_t(stream);
    uint16_t interval_ms;
    if (!chan_interval(chan, nav_msg[s], interval_ms)) {
        return false;
    }
    if (comm_get_txspace(mavlink_channel_t(chan)) < uint32_t(payload_bytes) + NAV_TX_HEADROOM_BYTES) {
        return false;
    }
    return _nav_cap[s][chan].take(interval_ms);
}

void AP_ARRC_SDR::nav_send(NavStream stream, mavlink_channel_t chan)
{
    switch (stream) {
    case NavStream::IMU: {
        const AP_InertialSensor &ins = AP::ins();
        const uint8_t i = 0;
        if (ins.get_gyro_count() == 0 || ins.get_accel_count() == 0 || ins.get_last_update_usec() == 0 ||
            !HAVE_PAYLOAD_SPACE(chan, ARRC_NAV_IMU)) {
            return;
        }
        const uint64_t t_us = sample_time_us(AP_HAL::micros64(), ins.get_last_update_usec());
        const Vector3f &gyro = ins.get_gyro(i);
        const Vector3f &accel = ins.get_accel(i);
        const float g[3] {gyro.x, gyro.y, gyro.z};
        const float a[3] {accel.x, accel.y, accel.z};
        const int16_t temperature = int16_t(constrain_float(ins.get_temperature(i) * 100.0f, INT16_MIN, INT16_MAX));
        const uint8_t flags = (ins.get_gyro_health(i) ? ARRC_NAV_IMU_FLAGS_GYRO_HEALTHY : 0) |
                              (ins.get_accel_health(i) ? ARRC_NAV_IMU_FLAGS_ACCEL_HEALTHY : 0);
        // the sums since the channel's previous sample, then a new interval from here
        ImuDelta &d = _imu_delta[chan];
        const uint32_t now_ms = AP_HAL::millis();
        Vector3f angle;
        Vector3f dv;
        float dt = 0;
        if (d.sent_ms != 0 && now_ms - d.sent_ms <= NAV_IMU_DELTA_IDLE_MS) {
            d.q.to_axis_angle(angle);
            dv = d.dv;
            dt = d.dt;
        }
        d.q.initialise();
        d.dv.zero();
        d.dt = 0;
        d.sent_ms = MAX(now_ms, 1U);
        const float da[3] {angle.x, angle.y, angle.z};
        const float dvel[3] {dv.x, dv.y, dv.z};
        mavlink_msg_arrc_nav_imu_send(chan, t_us, g, a, temperature, i, flags, da, dvel, dt);
        break;
    }
    case NavStream::MAG: {
        const Compass &compass = AP::compass();
        const uint8_t i = 0;
        if (compass.get_count() == 0 || compass.last_update_usec(i) == 0 || !HAVE_PAYLOAD_SPACE(chan, ARRC_NAV_MAG)) {
            return;
        }
        const uint64_t t_us = sample_time_us(AP_HAL::micros64(), compass.last_update_usec(i));
        const Vector3f &field = compass.get_field(i);
        const float f[3] {field.x, field.y, field.z};
        mavlink_msg_arrc_nav_mag_send(chan, t_us, f, i, compass.healthy(i) ? 1 : 0);
        break;
    }
    case NavStream::BARO: {
        const AP_Baro &baro = AP::baro();
        const uint8_t i = 0;
        if (baro.num_instances() == 0 || baro.get_last_update(i) == 0 || !HAVE_PAYLOAD_SPACE(chan, ARRC_NAV_BARO)) {
            return;
        }
        const uint64_t t_us = sample_time_ms_to_us(AP_HAL::micros64(), baro.get_last_update(i));
        mavlink_msg_arrc_nav_baro_send(chan, t_us, baro.get_pressure(i), baro.get_altitude(i),
                                       baro.get_temperature(i), i, baro.healthy(i) ? 1 : 0);
        break;
    }
    case NavStream::GPS:
        if (HAVE_PAYLOAD_SPACE(chan, ARRC_NAV_GPS)) {
            send_nav_gps(chan);
        }
        break;
    case NavStream::ECEF:
        if (HAVE_PAYLOAD_SPACE(chan, ARRC_NAV_ECEF)) {
            send_nav_ecef(chan);
        }
        break;
    case NavStream::GIMBAL: {
#if HAL_MOUNT_ENABLED
        AP_Mount *mount = AP::mount();
        if (mount != nullptr) {
            mount->send_ARRC_pointing(chan);
        }
#endif
        break;
    }
    case NavStream::COUNT:
        break;
    }
}

void AP_ARRC_SDR::nav_imu_sample(void)
{
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        if (nav_ready(NavStream::IMU, c, PAYLOAD_SIZE(mavlink_channel_t(c), ARRC_NAV_IMU))) {
            nav_send(NavStream::IMU, mavlink_channel_t(c));
        }
    }
}

void AP_ARRC_SDR::nav_imu_accumulate(void)
{
    const AP_InertialSensor &ins = AP::ins();
    const uint32_t update_us = ins.get_last_update_usec();
    if (update_us == _imu_delta_update_us) {
        return;
    }
    _imu_delta_update_us = update_us;
    const uint8_t i = 0;
    Vector3f da;
    Vector3f dv;
    float da_dt;
    float dv_dt;
    if (!ins.get_delta_angle(i, da, da_dt) || !ins.get_delta_velocity(i, dv, dv_dt)) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t c = 0; c < MIN(gcs().num_gcs(), uint8_t(MAVLINK_COMM_NUM_BUFFERS)); c++) {
        ImuDelta &d = _imu_delta[c];
        if (d.sent_ms == 0 || now_ms - d.sent_ms > NAV_IMU_DELTA_IDLE_MS) {
            continue;
        }
        // as EKF3 down-samples its IMU data: the rotation as a quaternion (summing angles would add coning
        // errors), each velocity change rotated into the body frame at the start of the interval
        d.q.rotate(da);
        d.q.normalize();
        Matrix3f rot;
        d.q.rotation_matrix(rot);
        d.dv += rot * dv;
        d.dt += da_dt;
    }
}

void AP_ARRC_SDR::nav_mag_sample(void)
{
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        if (nav_ready(NavStream::MAG, c, PAYLOAD_SIZE(mavlink_channel_t(c), ARRC_NAV_MAG))) {
            nav_send(NavStream::MAG, mavlink_channel_t(c));
        }
    }
}

void AP_ARRC_SDR::nav_baro_sample(void)
{
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        if (nav_ready(NavStream::BARO, c, PAYLOAD_SIZE(mavlink_channel_t(c), ARRC_NAV_BARO))) {
            nav_send(NavStream::BARO, mavlink_channel_t(c));
        }
    }
}

void AP_ARRC_SDR::nav_gimbal_sample(void)
{
    for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
        if (nav_ready(NavStream::GIMBAL, c, PAYLOAD_SIZE(mavlink_channel_t(c), ARRC_GIMBAL_POINTING))) {
            nav_send(NavStream::GIMBAL, mavlink_channel_t(c));
        }
    }
}

void AP_ARRC_SDR::nav_gps_sample(void)
{
    const AP_GPS &gps = AP::gps();
    const uint8_t i = 0;
    if (gps.status(i) != AP_GPS::NO_GPS && gps.last_message_time_ms(i) != 0) {
        const uint64_t fix_us = sample_time_ms_to_us(AP_HAL::micros64(), gps.last_message_time_ms(i));
        if (fix_us != _gps_fix_us) {
            _gps_fix_us = fix_us;
            for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
                if (nav_ready(NavStream::GPS, c, PAYLOAD_SIZE(mavlink_channel_t(c), ARRC_NAV_GPS))) {
                    send_nav_gps(mavlink_channel_t(c));
                }
            }
        }
    }

    AP_GPS::GPS_HPPOSECEF e;
    if (AP::gps().get_hpposecef_latest(i, e) && e.time_us != _ecef_fix_us) {
        _ecef_fix_us = e.time_us;
        for (uint8_t c = 0; c < gcs().num_gcs(); c++) {
            if (nav_ready(NavStream::ECEF, c, PAYLOAD_SIZE(mavlink_channel_t(c), ARRC_NAV_ECEF))) {
                send_nav_ecef(mavlink_channel_t(c));
            }
        }
    }
}

void AP_ARRC_SDR::send_nav_gps(mavlink_channel_t chan)
{
    const AP_GPS &gps = AP::gps();
    const uint8_t i = 0;
    if (gps.status(i) == AP_GPS::NO_GPS || gps.last_message_time_ms(i) == 0) {
        return;
    }
    const uint64_t t_us = sample_time_ms_to_us(AP_HAL::micros64(), gps.last_message_time_ms(i));
    const Location &loc = gps.location(i);
    const Vector3f &vel = gps.velocity(i);
    const float v[3] {vel.x, vel.y, vel.z};
    uint8_t flags = gps.have_vertical_velocity(i) ? ARRC_NAV_GPS_FLAGS_VEL_D_VALID : 0;
    float undulation = 0;
    int32_t alt_ellipsoid_mm = 0;
    if (gps.get_undulation(i, undulation)) {
        alt_ellipsoid_mm = loc.alt * 10 - undulation * 1000;    // as GPA.AEl
        flags |= ARRC_NAV_GPS_FLAGS_ALT_ELLIPSOID_VALID;
    }
    float h_acc = NaNf;
    float v_acc = NaNf;
    float s_acc = NaNf;
    // the driver's corrected fix time (receiver output, jitter corrected), if it belongs to this fix
    uint64_t epoch_us = gps.last_corrected_gps_time_usec(i);
    if (epoch_us == 0 || epoch_us > t_us + 1000U || t_us + 1000U - epoch_us > NAV_GPS_EPOCH_MAX_AGE_US + 1000U) {
        epoch_us = 0;
    }
    if (gps.horizontal_accuracy(i, h_acc)) {
        flags |= ARRC_NAV_GPS_FLAGS_H_ACC_VALID;
    }
    if (gps.vertical_accuracy(i, v_acc)) {
        flags |= ARRC_NAV_GPS_FLAGS_V_ACC_VALID;
    }
    if (gps.speed_accuracy(i, s_acc)) {
        flags |= ARRC_NAV_GPS_FLAGS_S_ACC_VALID;
    }
    mavlink_msg_arrc_nav_gps_send(chan, t_us, gps.time_week_ms(i), loc.lat, loc.lng, loc.alt * 10, alt_ellipsoid_mm,
                                  v, h_acc, v_acc, s_acc, gps.time_week(i), gps.get_hdop(i), gps.get_vdop(i),
                                  uint8_t(gps.status(i)), gps.num_sats(i), i, flags, epoch_us);
}

void AP_ARRC_SDR::send_nav_ecef(mavlink_channel_t chan)
{
    const uint8_t i = 0;
    AP_GPS::GPS_HPPOSECEF e;
    if (!AP::gps().get_hpposecef_latest(i, e)) {
        return;
    }
    const int32_t ecef[3] {e.ecef_x, e.ecef_y, e.ecef_z};
    const int8_t ecef_hp[3] {e.ecef_x_hp, e.ecef_y_hp, e.ecef_z_hp};
    mavlink_msg_arrc_nav_ecef_send(chan, e.time_us, e.itow, ecef, e.p_acc, e.gps_week, ecef_hp, e.flags, i);
}

#endif  // AP_ARRC_SDR_ENABLED
