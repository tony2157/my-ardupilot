#include "Copter.h"

void Copter::init_ARRC_lb5900(){
    if (arrc_rf_source() == ARRCRFSource::SDR) {
        // SDR only: never poll the LB5900
        ARRC_LB5900.set_enabled(false);
        return;
    }
    // Start the sensor: it is configured from the I2C bus thread, so boot does not wait for it
    uint8_t i2cAddr = g2.user_parameters.get_lb5900_address();
    uint16_t freq = g2.user_parameters.get_lb5900_freq();
    uint8_t avg_cnt = g2.user_parameters.get_lb5900_avg_cnt();
    uint8_t rate = g2.user_parameters.get_lb5900_mrate();
    uint8_t busId = g2.user_parameters.get_lb5900_busId();
    arrc_lb.init_ms = AP_HAL::millis();
    if (!ARRC_LB5900.init(busId,i2cAddr,freq,avg_cnt,rate)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LB5900: no I2C bus %u", busId);
    }
}

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();
#if AP_ARRC_SDR_ENABLED
    // the companion's barometer samples, at the BARO log rate
    ARRC_SDR.nav_baro_sample();
#endif

    baro_alt_m = barometer.get_altitude();
}

#if AP_RANGEFINDER_ENABLED
void Copter::init_rangefinder(void)
{
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_m_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // upward facing range finder
   rangefinder_up_state.alt_m_filt.set_cutoff_frequency(g2.rangefinder_filt);
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
        g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_m_filt.get() * 100.0);
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
    float terrain_u_m = rangefinder_state.ref_pos_u_m - rangefinder_state.alt_glitch_protected_m;
    rangefinder_state.terrain_u_m += (terrain_u_m - rangefinder_state.terrain_u_m) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    terrain_u_m = rangefinder_up_state.ref_pos_u_m + rangefinder_up_state.alt_glitch_protected_m;
    rangefinder_up_state.terrain_u_m += (terrain_u_m - rangefinder_up_state.terrain_u_m) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    if (rangefinder_state.alt_healthy || rangefinder_state.data_stale()) {
        wp_nav->set_rangefinder_terrain_U_m(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.terrain_u_m);
#if MODE_CIRCLE_ENABLED
        circle_nav->set_rangefinder_terrain_U_m(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.terrain_u_m);
#endif
    }
}

// helper function to get inertially interpolated rangefinder height.
bool Copter::get_rangefinder_height_interpolated_m(float& height_m) const
{
#if AP_RANGEFINDER_ENABLED
    return rangefinder_state.get_rangefinder_height_interpolated_m(height_m);
#else
    return false;
#endif
}
