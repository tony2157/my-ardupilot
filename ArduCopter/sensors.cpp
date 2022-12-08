#include "Copter.h"

void Copter::init_CASS_imet(){

    float coeff[4][4];

    //CS3D SENSORS (dummy values)
    //IMET temp number 57560:
    coeff[0][0] = 9.98873354e-04f;
    coeff[0][1] = 2.63219974e-04f;
    coeff[0][2] = 0.0f;
    coeff[0][3] = 1.47120693e-07f;

    //IMET temp number 57551:
    coeff[1][0] = 1.02017189e-03f;
    coeff[1][1] = 2.60496203e-04f;
    coeff[1][2] = 0.0f;
    coeff[1][3] = 1.52569843e-07f;

    //IMET temp number 57558:
    coeff[2][0] = 1.01048989e-03f;
    coeff[2][1] = 2.62050421e-04f;
    coeff[2][2] = 0.0f;
    coeff[2][3] = 1.48891207e-07f;

    //IMET temp number none:
    coeff[3][0] = 1.0f;
    coeff[3][1] = 1.0f;
    coeff[3][2] = 0.0f;
    coeff[3][3] = 1.0f;
    
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
}

#if AP_RANGEFINDER_ENABLED
void Copter::init_rangefinder(void)
{
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // upward facing range finder
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
    rangefinder.update();

    rangefinder_state.update();
    rangefinder_up_state.update();

#if HAL_PROXIMITY_ENABLED
    if (rangefinder_state.enabled_and_healthy() || rangefinder_state.data_stale()) {
        g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
    }
#endif
}
#endif  // AP_RANGEFINDER_ENABLED

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok() const
{
    return rangefinder_state.enabled_and_healthy();
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok() const
{
    return rangefinder_up_state.enabled_and_healthy();
}

// update rangefinder based terrain offset
// terrain offset is the terrain's height above the EKF origin
void Copter::update_rangefinder_terrain_offset()
{
    float terrain_offset_cm = rangefinder_state.inertial_alt_cm - rangefinder_state.alt_cm_glitch_protected;
    rangefinder_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    terrain_offset_cm = rangefinder_up_state.inertial_alt_cm + rangefinder_up_state.alt_cm_glitch_protected;
    rangefinder_up_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_up_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    if (rangefinder_state.alt_healthy || rangefinder_state.data_stale()) {
        wp_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#if MODE_CIRCLE_ENABLED
        circle_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#endif
    }
}

// helper function to get inertially interpolated rangefinder height.
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret) const
{
#if AP_RANGEFINDER_ENABLED
    return rangefinder_state.get_rangefinder_height_interpolated_cm(ret);
#else
    return false;
#endif
}
