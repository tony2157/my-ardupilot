#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>

void Plane::init_CASS_imet(){

    float coeff[4][3];

    // //CS2.2 COPTERSONDE SENSORS
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
    
    //CS2.1 COPTERSONDE SENSORS
    //IMET temp number 56238:
    coeff[0][0] = 9.99731794e-04f;
    coeff[0][1] = 2.63482606e-04f;
    coeff[0][2] = 1.46874135e-07f;

    //IMET temp number 56237:
    coeff[1][0] = 1.00379726e-03f;
    coeff[1][1] = 2.62770143e-04f;
    coeff[1][2] = 1.48291724e-07f;

    //IMET temp number 56239:
    coeff[2][0] = 9.99173098e-04f;
    coeff[2][1] = 2.63582305e-04f;
    coeff[2][2] = 1.46930574e-07f;

    //IMET temp number none:
    coeff[3][0] = 1.0;
    coeff[3][1] = 1.0;
    coeff[3][2] = 1.0;

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

    for(uint8_t i=0; i<4; i++){
        CASS_Imet[i].init();
    }
    // Set I2C addresses
    CASS_Imet[1].set_i2c_addr(0x4B);
    CASS_Imet[2].set_i2c_addr(0x4A);
    CASS_Imet[3].set_i2c_addr(0x49);
    // Set sensor coefficients
    CASS_Imet[0].set_sensor_coeff(coeff[0]);
    CASS_Imet[1].set_sensor_coeff(coeff[1]);
    CASS_Imet[2].set_sensor_coeff(coeff[2]);
    CASS_Imet[3].set_sensor_coeff(coeff[3]);
}

void Plane::init_CASS_hyt271(){
    for(uint8_t i=0; i<4; i++){
        CASS_HYT271[i].init();
    }
    // Set I2C addresses
    CASS_HYT271[1].set_i2c_addr(0x11);
    CASS_HYT271[2].set_i2c_addr(0x12);
    CASS_HYT271[3].set_i2c_addr(0x13);
}

void Plane::init_CASS_co2(){
    CASS_CO2[0].initUART0();
    CASS_CO2[1].initUART1();
}

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{

    // notify the rangefinder of our approximate altitude above ground to allow it to power on
    // during low-altitude flight when configured to power down during higher-altitude flight
    float height;
#if AP_TERRAIN_AVAILABLE
    if (terrain.status() == AP_Terrain::TerrainStatusOK && terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    } else
#endif
    {
        // use the best available alt estimate via baro above home
        if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
            // ensure the rangefinder is powered-on when land alt is higher than home altitude.
            // This is done using the target alt which we know is below us and we are sinking to it
            height = height_above_target();
        } else {
            // otherwise just use the best available baro estimate above home.
            height = relative_altitude;
        }
        rangefinder.set_estimated_terrain_height(height);
    }

    rangefinder.update();

    if ((rangefinder.num_sensors() > 0) && should_log(MASK_LOG_SONAR)) {
        Log_Write_Sonar();
    }

    rangefinder_height_update();
}

/*
  calibrate compass
*/
void Plane::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

/*
    Accel calibration
*/
void Plane::accel_cal_update() {
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}

/*
  ask airspeed sensor for a new value
 */
void Plane::read_airspeed(void)
{
    if (airspeed.enabled()) {
        airspeed.read();
        if (should_log(MASK_LOG_IMU)) {
            DataFlash.Log_Write_Airspeed(airspeed);
        }

        // supply a new temperature to the barometer from the digital
        // airspeed sensor if we can
        float temperature;
        if (airspeed.get_temperature(temperature)) {
            barometer.set_external_temperature(temperature);
        }
    }

    // we calculate airspeed errors (and thus target_airspeed_cm) even
    // when airspeed is disabled as TECS may be using synthetic
    // airspeed for a quadplane transition
    calc_airspeed_errors();
    
    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(&aspeed)) {
        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
    }
}

/*
  update RPM sensors
 */
void Plane::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RC)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Plane::update_sensor_status_flags(void)
{
     // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }

    if (airspeed.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (geofence_present()) {
        control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
    }

    if (aparm.throttle_min < 0) {
        control_sensors_present |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }
    if (plane.DataFlash.logging_present()) { // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }

    // all present sensors enabled by default except rate control, attitude stabilization, yaw, altitude, position control, geofence, motor, and battery output which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL & ~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION & ~MAV_SYS_STATUS_SENSOR_YAW_POSITION & ~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL & ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS & ~MAV_SYS_STATUS_GEOFENCE & ~MAV_SYS_STATUS_LOGGING & ~MAV_SYS_STATUS_SENSOR_BATTERY);

    if (airspeed.enabled() && airspeed.use()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }

    if (geofence_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
    }

    if (plane.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }

    switch (control_mode) {
    case MANUAL:
        break;

    case ACRO:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        break;

    case STABILIZE:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case QSTABILIZE:
    case QHOVER:
    case QLAND:
    case QLOITER:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case FLY_BY_WIRE_B:
    case CRUISE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case TRAINING:
        if (!training_manual_roll || !training_manual_pitch) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation        
        }
        break;

    case AUTO:
    case RTL:
    case LOITER:
    case AVOID_ADSB:
    case GUIDED:
    case CIRCLE:
    case QRTL:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL; // altitude control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
        break;

    case INITIALISING:
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // default: all present sensors healthy except baro, 3D_MAG, GPS, DIFFERNTIAL_PRESSURE.   GEOFENCE always defaults to healthy.
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                         MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                         MAV_SYS_STATUS_SENSOR_GPS |
                                                         MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
    control_sensors_health |= MAV_SYS_STATUS_GEOFENCE;

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }
    if (ahrs.have_inertial_nav() && !ins.accel_calibrated_ok_all()) {
        // trying to use EKF without properly calibrated accelerometers
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (barometer.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.is_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }
    if (airspeed.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
#if GEOFENCE_ENABLED
    if (geofence_breached()) {
        control_sensors_health &= ~MAV_SYS_STATUS_GEOFENCE;
    }
#endif

    if (plane.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

    if (millis() - failsafe.last_valid_rc_ms < 200) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    } else {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

#if AP_TERRAIN_AVAILABLE
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

    if (rangefinder.has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (g.rangefinder_landing) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        if (rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;            
        }
    }

    if (aparm.throttle_min < 0 && SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) < 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_REVERSE_MOTOR;
        control_sensors_health |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }

    if (AP_Notify::flags.initialising) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    if (!plane.battery.healthy() || plane.battery.has_failsafed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_BATTERY;
    }

#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}
