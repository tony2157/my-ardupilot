/*
 * AP_ARRC_SDR sensor sample streams to the companion computer (ARRC_NAV_*, ARRC_GIMBAL_POINTING).
 *
 * The companion records them in its HDF5 data next to the SDR data, so the data can be processed without
 * the autopilot log. Each stream runs at the rate the autopilot logs the same data: IMU, MAG, BARO and ARGM
 * samples are sent from the vehicle tasks that log them (the IMU follows LOG_BITMASK IMU / IMU_FAST), GPS
 * and ECEF once per new fix. That cadence holds whether or not the log is being written. The companion
 * requests each message with MAV_CMD_SET_MESSAGE_INTERVAL on its link: that only subscribes the link.
 * Instance 0 of each sensor; samples carry the time they were taken on the boot clock.
 */
#include "AP_ARRC_SDR.h"

#if AP_ARRC_SDR_ENABLED

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Mount/AP_Mount.h>
#include <GCS_MAVLink/GCS.h>

// a channel keeps a stream this long after its message was last due there
static constexpr uint32_t NAV_SUBSCRIPTION_MS = 2000;

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
    _nav_due_ms[uint8_t(stream)][chan] = MAX(AP_HAL::millis(), 1U);
    if (stream == NavStream::GPS) {
        send_nav_gps(chan);
    } else if (stream == NavStream::ECEF) {
        send_nav_ecef(chan);
    }
}

bool AP_ARRC_SDR::nav_subscribed(NavStream stream, uint8_t chan) const
{
    const uint32_t due_ms = _nav_due_ms[uint8_t(stream)][chan];
    return due_ms != 0 && AP_HAL::millis() - due_ms < NAV_SUBSCRIPTION_MS;
}

void AP_ARRC_SDR::nav_imu_sample(void)
{
    const AP_InertialSensor &ins = AP::ins();
    const uint8_t i = 0;
    if (ins.get_gyro_count() == 0 || ins.get_accel_count() == 0 || ins.get_last_update_usec() == 0) {
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
    for (uint8_t c = 0; c < MAVLINK_COMM_NUM_BUFFERS; c++) {
        const mavlink_channel_t chan = mavlink_channel_t(c);
        if (nav_subscribed(NavStream::IMU, c) && HAVE_PAYLOAD_SPACE(chan, ARRC_NAV_IMU)) {
            mavlink_msg_arrc_nav_imu_send(chan, t_us, g, a, temperature, i, flags);
        }
    }
}

void AP_ARRC_SDR::nav_mag_sample(void)
{
    const Compass &compass = AP::compass();
    const uint8_t i = 0;
    if (compass.get_count() == 0 || compass.last_update_usec(i) == 0) {
        return;
    }
    const uint64_t t_us = sample_time_us(AP_HAL::micros64(), compass.last_update_usec(i));
    const Vector3f &field = compass.get_field(i);
    const float f[3] {field.x, field.y, field.z};
    for (uint8_t c = 0; c < MAVLINK_COMM_NUM_BUFFERS; c++) {
        const mavlink_channel_t chan = mavlink_channel_t(c);
        if (nav_subscribed(NavStream::MAG, c) && HAVE_PAYLOAD_SPACE(chan, ARRC_NAV_MAG)) {
            mavlink_msg_arrc_nav_mag_send(chan, t_us, f, i, compass.healthy(i) ? 1 : 0);
        }
    }
}

void AP_ARRC_SDR::nav_baro_sample(void)
{
    const AP_Baro &baro = AP::baro();
    const uint8_t i = 0;
    if (baro.num_instances() == 0 || baro.get_last_update(i) == 0) {
        return;
    }
    const uint64_t t_us = sample_time_ms_to_us(AP_HAL::micros64(), baro.get_last_update(i));
    for (uint8_t c = 0; c < MAVLINK_COMM_NUM_BUFFERS; c++) {
        const mavlink_channel_t chan = mavlink_channel_t(c);
        if (nav_subscribed(NavStream::BARO, c) && HAVE_PAYLOAD_SPACE(chan, ARRC_NAV_BARO)) {
            mavlink_msg_arrc_nav_baro_send(chan, t_us, baro.get_pressure(i), baro.get_altitude(i),
                                           baro.get_temperature(i), i, baro.healthy(i) ? 1 : 0);
        }
    }
}

void AP_ARRC_SDR::nav_gimbal_sample(void)
{
#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    for (uint8_t c = 0; c < MAVLINK_COMM_NUM_BUFFERS; c++) {
        if (nav_subscribed(NavStream::GIMBAL, c)) {
            mount->send_ARRC_pointing(mavlink_channel_t(c));
        }
    }
#endif
}

void AP_ARRC_SDR::send_nav_gps(mavlink_channel_t chan)
{
    const AP_GPS &gps = AP::gps();
    const uint8_t i = 0;
    if (gps.status(i) == AP_GPS::NO_GPS || gps.last_message_time_ms(i) == 0) {
        return;
    }
    const uint64_t t_us = sample_time_ms_to_us(AP_HAL::micros64(), gps.last_message_time_ms(i));
    if (t_us == _gps_sent_us[chan]) {
        return;
    }
    _gps_sent_us[chan] = t_us;
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
                                  uint8_t(gps.status(i)), gps.num_sats(i), i, flags);
}

void AP_ARRC_SDR::send_nav_ecef(mavlink_channel_t chan)
{
    const uint8_t i = 0;
    AP_GPS::GPS_HPPOSECEF e;
    if (!AP::gps().get_hpposecef_latest(i, e) || e.time_us == _ecef_sent_us[chan]) {
        return;
    }
    _ecef_sent_us[chan] = e.time_us;
    const int32_t ecef[3] {e.ecef_x, e.ecef_y, e.ecef_z};
    const int8_t ecef_hp[3] {e.ecef_x_hp, e.ecef_y_hp, e.ecef_z_hp};
    mavlink_msg_arrc_nav_ecef_send(chan, e.time_us, e.itow, ecef, e.p_acc, e.gps_week, ecef_hp, e.flags, i);
}

#endif  // AP_ARRC_SDR_ENABLED
