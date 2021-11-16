#include "Copter.h"

void Copter::init_CASS_imet(){

    float coeff[4][4];

    // //CS2 BILLSONDE SENSORS
    // //IMET temp number 56236:
    // coeff[0][0] = 1.00993256e-03f;
    // coeff[0][1] = 2.62155349e-04f;
    // coeff[0][2] = 1.48648209e-07f;

    // //IMET temp number 45364:
    // coeff[1][0] = 1.01307391e-03f;
    // coeff[1][1] = 2.61020114e-04f;
    // coeff[1][2] = 1.52660155e-07f;

    // //IMET temp number 56220:
    // coeff[2][0] = 1.01704202e-03f;
    // coeff[2][1] = 2.61212821e-04f;
    // coeff[2][2] = 1.51670179e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    // //Scoop E SENSORS
    // //IMET temp number 62321:
    // coeff[0][0] = 1.01649866e-03f;
    // coeff[0][1] = 2.60868443e-04f;
    // coeff[0][2] = 1.51641342e-07f;

    // //IMET temp number 62289:
    // coeff[1][0] = 1.00537764e-03f;
    // coeff[1][1] = 2.62828696e-04f;
    // coeff[1][2] = 1.46265555e-07f;

    // //IMET temp number 62299:
    // coeff[2][0] = 1.01656967e-03f;
    // coeff[2][1] = 2.60866688e-04f;
    // coeff[2][2] = 1.51720394e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    // //Scoop F SENSORS
    // //IMET temp number 62275:
    // coeff[0][0] = 1.01689169e-03f;
    // coeff[0][1] = 2.60664507e-04f;
    // coeff[0][2] = 1.51703287e-07f;

    // //IMET temp number 62288:
    // coeff[1][0] = 1.01009058e-03f;
    // coeff[1][1] = 2.61933278e-04f;
    // coeff[1][2] = 1.51703287e-07f;

    // //IMET temp number 62298:
    // coeff[2][0] = 1.01624431e-03f;
    // coeff[2][1] = 2.60956943e-04f;
    // coeff[2][2] = 1.51882518e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    // //Scoop G SENSORS
    // //IMET temp number 62307:
    // coeff[0][0] = 1.00715675e-03f;
    // coeff[0][1] = 2.62263287e-04f;
    // coeff[0][2] = 0.0f;
    // coeff[0][3] = 1.48208835e-07f;

    // //IMET temp number 62322:
    // coeff[1][0] = 1.01690278e-03f;
    // coeff[1][1] = 2.60964166e-04f;
    // coeff[1][2] = 0.0f;
    // coeff[1][3] = 1.50972422e-07f;

    // //IMET temp number 62290:
    // coeff[2][0] = 1.01401588e-03f;
    // coeff[2][1] = 2.61236308e-04f;
    // coeff[2][2] = 0.0f;
    // coeff[2][3] = 1.50931780e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 0.0f;
    // coeff[3][3] = 1.0;

    // //Scoop H SENSORS
    // //IMET temp number 62312:
    // coeff[0][0] = 1.01569553e-03f;
    // coeff[0][1] = 2.61087436e-04f;
    // coeff[0][2] = 1.50792685e-07f;

    // //IMET temp number 62323:
    // coeff[1][0] = 1.01697122e-03f;
    // coeff[1][1] = 2.61029242e-04f;
    // coeff[1][2] = 1.51370375e-07f;

    // //IMET temp number 62291:
    // coeff[2][0] = 1.01576578e-03f;
    // coeff[2][1] = 2.61030485e-04f;
    // coeff[2][2] = 1.51113357e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    //Scoop L SENSORS
    //IMET temp number 62270:
    coeff[0][0] = 1.01203424e-03f;
    coeff[0][1] = 2.61424147e-04f;
    coeff[0][2] = 0.0f;
    coeff[0][3] = 1.50496506e-07f;

    //IMET temp number 62313:
    coeff[1][0] = 1.01790856e-03f;
    coeff[1][1] = 2.60841385e-04f;
    coeff[1][2] = 0.0f;
    coeff[1][3] = 1.51556507e-07f;

    //IMET temp number 62296:
    coeff[2][0] = 1.01891220e-03f;
    coeff[2][1] = 2.60798714e-04f;
    coeff[2][2] = 0.0f;
    coeff[2][3] = 1.51768316e-07f;

    //IMET temp number none:
    coeff[3][0] = 1.0;
    coeff[3][1] = 1.0;
    coeff[3][2] = 0.0f;
    coeff[3][3] = 1.0;

    // //CS3D SENSORS
    // //IMET temp number 57560:
    // coeff[0][0] = 9.98873354e-04f;
    // coeff[0][1] = 2.63219974e-04f;
    // coeff[0][2] = 1.47120693e-07f;

    // //IMET temp number 57551:
    // coeff[1][0] = 1.02017189e-03f;
    // coeff[1][1] = 2.60496203e-04f;
    // coeff[1][2] = 1.52569843e-07f;

    // //IMET temp number 57558:
    // coeff[2][0] = 1.01048989e-03f;
    // coeff[2][1] = 2.62050421e-04f;
    // coeff[2][2] = 1.48891207e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;
    
    // //CS2.5 TONYSONDE SENSORS
    // //IMET temp number 57562:
    // coeff[0][0] = 1.02777010e-03f;
    // coeff[0][1] = 2.59349232e-04f;
    // coeff[0][2] = 1.56043078e-07f;

    // //IMET temp number 57563:
    // coeff[1][0] = 9.91077399e-04f;
    // coeff[1][1] = 2.64646362e-04f;
    // coeff[1][2] = 1.43596294e-07f;

    // //IMET temp number 58821:
    // coeff[2][0] = 1.00786813e-03f;
    // coeff[2][1] = 2.61722397e-04f;
    // coeff[2][2] = 1.48476183e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    // //CS2.5 CHRISSONDE SENSORS
    // //IMET temp number 57549:
    // coeff[0][0] = 1.01991700e-03f;
    // coeff[0][1] = 2.60780405e-04f;
    // coeff[0][2] = 1.52712244e-07f;

    // //IMET temp number 58814:
    // coeff[1][0] = 1.01248342e-03f;
    // coeff[1][1] = 2.61214144e-04f;
    // coeff[1][2] = 1.50257257e-07f;

    // //IMET temp number 58822:
    // coeff[2][0] = 1.00761568e-03f;
    // coeff[2][1] = 2.61899525e-04f;
    // coeff[2][2] = 1.49075281e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    // //CS2.5 DEREKSONDE SENSORS
    // //IMET temp number 57555:
    // coeff[0][0] = 1.02009822e-03f;
    // coeff[0][1] = 2.60475129e-04f;
    // coeff[0][2] = 1.54057907e-07f;

    // //IMET temp number 57564:
    // coeff[1][0] = 1.01554852e-03f;
    // coeff[1][1] = 2.61642038e-04f;
    // coeff[1][2] = 1.50140249e-07f;

    // //IMET temp number 57565:
    // coeff[2][0] = 1.02812678e-03f;
    // coeff[2][1] = 2.59630119e-04f;
    // coeff[2][2] = 1.55078895e-07f;

    // //IMET temp number none:
    // coeff[3][0] = 1.0;
    // coeff[3][1] = 1.0;
    // coeff[3][2] = 1.0;

    // //RED COPTERSONDE SENSORS
    // //IMET temp number 48623:
    // coeff[0][0] = 1.00733068f * (float)pow(10, -3);
    // coeff[0][1] = 2.62299300f * (float)pow(10, -4);
    // coeff[0][2] = 1.48361439f * (float)pow(10, -7);

    // //IMET temp number 48627:
    // coeff[1][0] = 1.00097308f * (float)pow(10, -3);
    // coeff[1][1] = 2.62806129f * (float)pow(10, -4);
    // coeff[1][2] = 1.46350112f * (float)pow(10, -7);

    // //IMET temp number 45363:
    // coeff[2][0] = 9.93118592f * (float)pow(10, -4);
    // coeff[2][1] = 2.63743049f * (float)pow(10, -4);
    // coeff[2][2] = 1.47415476f * (float)pow(10, -7);

    // //IMET temp number 48622:
    // coeff[3][0] = 1.00880279f * (float)pow(10, -3);
    // coeff[3][1] = 2.61500024f * (float)pow(10, -4);
    // coeff[3][2] = 1.49421629f * (float)pow(10, -7);

    // Initialize and set I2C addresses
    uint8_t deafult_i2cAddr = 0x48;
    uint8_t busId = 0;
    for(uint8_t i=0; i<4; i++){
        CASS_Imet[i].init(busId,deafult_i2cAddr + i);
    }
    // Set sensor coefficients
    CASS_Imet[0].set_sensor_coeff(coeff[0]);
    CASS_Imet[1].set_sensor_coeff(coeff[1]);
    CASS_Imet[2].set_sensor_coeff(coeff[2]);
    CASS_Imet[3].set_sensor_coeff(coeff[3]);
}

void Copter::init_CASS_hyt271(){
    // Initialize and set I2C addresses
    uint8_t deafult_i2cAddr = 0x10;
    uint8_t busId = 0;
    for(uint8_t i=0; i<4; i++){
        CASS_HYT271[i].init(busId,deafult_i2cAddr + i);
    }
}

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.init();
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    if (rangefinder.num_sensors() > 0 &&
        should_log(MASK_LOG_CTUN)) {
        DataFlash.Log_Write_RFND(rangefinder);
    }

    rangefinder_state.alt_healthy = ((rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::RangeFinder_Good) && (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt = rangefinder.distance_cm_orient(ROTATION_PITCH_270);

 #if RANGEFINDER_TILT_CORRECTION == ENABLED
    // correct alt for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
 #endif

    rangefinder_state.alt_cm = temp_alt;

    // filter rangefinder for use by AC_WPNav
    uint32_t now = AP_HAL::millis();

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
        } else {
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
    }

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());

#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
#if RPM_ENABLED == ENABLED
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
#endif
}

// initialise compass
void Copter::init_compass()
{
    if (!g.compass_enabled) {
        return;
    }

    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        hal.console->printf("COMPASS INIT ERROR\n");
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

/*
  if the compass is enabled then try to accumulate a reading
  also update initial location used for declination
 */
void Copter::compass_accumulate(void)
{
    if (!g.compass_enabled) {
        return;
    }

    compass.accumulate();

    // update initial location used for declination
    if (!ap.compass_init_location) {
        Location loc;
        if (ahrs.get_position(loc)) {
            compass.set_initial_location(loc.lat, loc.lng);
            ap.compass_init_location = true;
        }
    }
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // initialise optical flow sensor
    optflow.init();
#endif      // OPTFLOW == ENABLED
}

// called at 200hz
#if OPTFLOW == ENABLED
void Copter::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        const Vector3f &posOffset = optflow.get_pos_offset();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update, posOffset);
        if (g.log_bitmask & MASK_LOG_OPTFLOW) {
            Log_Write_Optflow();
        }
    }
}
#endif  // OPTFLOW == ENABLED

void Copter::compass_cal_update()
{
    static uint32_t compass_cal_stick_gesture_begin = 0;

    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }

    if (compass.is_calibrating()) {
        if (channel_yaw->get_control_in() < -4000 && channel_throttle->get_control_in() > 900) {
            compass.cancel_calibration_all();
        }
    } else {
        bool stick_gesture_detected = compass_cal_stick_gesture_begin != 0 && !motors->armed() && channel_yaw->get_control_in() > 4000 && channel_throttle->get_control_in() > 900;
        uint32_t tnow = millis();

        if (!stick_gesture_detected) {
            compass_cal_stick_gesture_begin = tnow;
        } else if (tnow-compass_cal_stick_gesture_begin > 1000*COMPASS_CAL_STICK_GESTURE_TIME) {
#ifdef CAL_ALWAYS_REBOOT
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,true);
#else
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,false);
#endif
        }
    }
}

void Copter::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }

#ifdef CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
    g2.proximity.set_rangefinder(&rangefinder);
#endif
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Copter::update_sensor_status_flags(void)
{
    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (copter.DataFlash.logging_present()) { // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif
#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder.has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif

    // all sensors are present except these, which may be set as enabled below:
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING &
                                                         ~MAV_SYS_STATUS_SENSOR_BATTERY &
                                                         ~MAV_SYS_STATUS_GEOFENCE &
                                                         ~MAV_SYS_STATUS_SENSOR_LASER_POSITION &
                                                         ~MAV_SYS_STATUS_SENSOR_PROXIMITY);

    switch (control_mode) {
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
    case RTL:
    case CIRCLE:
    case LAND:
    case POSHOLD:
    case BRAKE:
    case THROW:
    case SMART_RTL:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case ALT_HOLD:
    case GUIDED_NOGPS:
    case SPORT:
    case AUTOTUNE:
    case FLOWHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    default:
        // stabilize, acro, drift, and flip have no automatic x,y or z control (i.e. all manual)
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    if (copter.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
    }
#endif
#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif


    // default to all healthy
    control_sensors_health = control_sensors_present;

    if (!barometer.all_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (!g.compass_enabled || !compass.healthy() || !ahrs.use_compass()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (!gps.is_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ap.rc_receiver_present || failsafe.radio) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
#if OPTFLOW == ENABLED
    if (!optflow.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
#if PRECISION_LANDING == ENABLED
    if (precland.enabled() && !precland.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    if (g2.visual_odom.enabled() && !g2.visual_odom.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (copter.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.sensor_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (!rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    if (!copter.battery.healthy() || copter.battery.has_failsafed()) {
         control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#if AC_FENCE == ENABLED
    if (copter.fence.sys_status_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_GEOFENCE;
    }
#endif

#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}

// init visual odometry sensor
void Copter::init_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    g2.visual_odom.init();
#endif
}

// update visual odometry sensor
void Copter::update_visual_odom()
{
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    // check for updates
    if (g2.visual_odom.enabled() && (g2.visual_odom.get_last_update_ms() != visual_odom_last_update_ms)) {
        visual_odom_last_update_ms = g2.visual_odom.get_last_update_ms();
        float time_delta_sec = g2.visual_odom.get_time_delta_usec() / 1000000.0f;
        ahrs.writeBodyFrameOdom(g2.visual_odom.get_confidence(),
                                g2.visual_odom.get_position_delta(),
                                g2.visual_odom.get_angle_delta(),
                                time_delta_sec,
                                visual_odom_last_update_ms,
                                g2.visual_odom.get_pos_offset());
        // log sensor data
        DataFlash.Log_Write_VisualOdom(time_delta_sec,
                                       g2.visual_odom.get_angle_delta(),
                                       g2.visual_odom.get_position_delta(),
                                       g2.visual_odom.get_confidence());
    }
#endif
}

// winch and wheel encoder initialisation
void Copter::winch_init()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.init();
    g2.winch.init(&g2.wheel_encoder);
#endif
}

// winch and wheel encoder update
void Copter::winch_update()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.update();
    g2.winch.update();
#endif
}
