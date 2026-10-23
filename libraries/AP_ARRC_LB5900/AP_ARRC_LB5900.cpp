#include "AP_ARRC_LB5900.h"

#if AP_ARRC_LB5900_ENABLED

#include <utility>
#include <stdio.h>
#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AP_ARRC_LB5900::AP_ARRC_LB5900() :
    _dev(nullptr),
    _sample{NaNf, 0, 0},
    _have_sample(false),
    _state(State::STOPPED),
    _enabled(true),
    _ever_configured(false),
    _period_ms(0),
    _last_ok_ms(0),
    _config_attempt_ms(0),
    _freq(0),
    _avg_cnt(1),
    _rate(0)
{
}

bool AP_ARRC_LB5900::init(uint8_t busId, uint8_t i2cAddr, uint16_t freq, uint8_t avg_cnt, uint8_t rate)
{
    // Store configuration
    _rate = (rate > 3) ? 3 : rate;
    _avg_cnt = (_rate == 2) ? 1 : MAX(avg_cnt, 1U);  // FAST mode forces avg_cnt to 1
    _freq = MIN(freq, 18000U);

    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    if (!_dev) {
        return false;
    }
    _dev->set_retries(3);

    // Calculate callback period based on MRATe and averaging count
    // Add 25% margin to ensure measurement is ready (per SPII2CHW.pdf p.4 recommendation)
    uint32_t measurement_period_us = _calculate_measurement_period_us(_rate, _avg_cnt);
    measurement_period_us = (measurement_period_us * 125) / 100;  // Add 25% margin

    // Clamp to reasonable bounds (minimum 5ms, maximum 500ms)
    measurement_period_us = MAX(measurement_period_us, 5000U);
    measurement_period_us = MIN(measurement_period_us, 500000U);
    _period_ms = measurement_period_us / 1000U;

    // the first callback sends the configuration
    _state = State::CONFIGURING;
    _dev->register_periodic_callback(measurement_period_us, FUNCTOR_BIND_MEMBER(&AP_ARRC_LB5900::_timer, void));

    return true;
}

void AP_ARRC_LB5900::set_enabled(bool enabled)
{
    _enabled = enabled;
    if (!enabled) {
        _state = State::STOPPED;
    } else if (_dev && _state == State::STOPPED) {
        // configure again: the sensor may have been reset while it was not polled
        _config_attempt_ms = 0;
        _state = State::CONFIGURING;
    }
}

bool AP_ARRC_LB5900::get_sample(Sample &sample) const
{
    WITH_SEMAPHORE(_sem);
    sample = _sample;
    return _have_sample;
}

float AP_ARRC_LB5900::power_measure(void) const
{
    WITH_SEMAPHORE(_sem);
    return _sample.power_dbm;
}

uint32_t AP_ARRC_LB5900::sample_count(void) const
{
    WITH_SEMAPHORE(_sem);
    return _sample.count;
}

bool AP_ARRC_LB5900::healthy(void) const
{
    if (!_enabled || _state != State::RUNNING) {
        return false;
    }
    Sample sample;
    if (!get_sample(sample)) {
        return false;
    }
    // a reading within the last three measurement periods (a few failed transfers in a row are tolerated)
    return AP_HAL::millis() - sample.time_ms < MAX(3U * _period_ms, 500U);
}

// send one SCPI command with the "next read is status and length" header
bool AP_ARRC_LB5900::_send_command(const char *cmd)
{
    const size_t len = strlen(cmd);
    if (len + 5 > sizeof(write_sensor_buffer.byte)) {
        return false;
    }
    memset(write_sensor_buffer.byte, 0x00, sizeof(write_sensor_buffer.byte));
    // header: next read type and the length (command, terminator and the 4 header bytes) big-endian
    const uint32_t total = len + 5;
    write_sensor_buffer.field.commandAndLength[0] = (uint8_t)nextReadIsStatusAndLength;
    write_sensor_buffer.field.commandAndLength[1] = (total >> 16) & 0xFF;
    write_sensor_buffer.field.commandAndLength[2] = (total >> 8) & 0xFF;
    write_sensor_buffer.field.commandAndLength[3] = total & 0xFF;
    memcpy(write_sensor_buffer.field.buffer, cmd, len);
    return _dev->transfer(write_sensor_buffer.byte, total, nullptr, 0);
}

bool AP_ARRC_LB5900::_configure(void)
{
    // Available sensor reading rate
    static const char *const mrate[4] {
        "NORMAL",   // 20 readings per sec
        "DOUBLE",   // 40 readings per sec
        "FAST",     // 110 readings per sec (disallows average count)
        "SUPER"     // 110 readings per sec (allows average count)
    };

    // Sized for the longest values: "FREQ 18000 MHZ", "SENS:AVER:COUN 255", "SENS:MRAT NORMAL"
    char FREQ[16];
    char AVG_CNT[20];
    char MRATE[20];
    snprintf(FREQ, sizeof(FREQ), "FREQ %u MHZ", (unsigned)_freq);
    snprintf(AVG_CNT, sizeof(AVG_CNT), "SENS:AVER:COUN %u", (unsigned)_avg_cnt);
    snprintf(MRATE, sizeof(MRATE), "SENS:MRAT %s", mrate[_rate]);

    // initial commands to configure the LB5900, the same the driver always sent
    const char *const cmd[] {
        "SYST:PRES DEF",
        "AVER:COUN:AUTO OFF",
        "SENS:AVER:SDET OFF",
        "INIT:CONT ON",
        MRATE,
        AVG_CNT,
        FREQ,
    };
    for (const char *c : cmd) {
        if (!_send_command(c)) {
            return false;
        }
    }
    return true;
}

uint32_t AP_ARRC_LB5900::_calculate_measurement_period_us(uint8_t rate, uint8_t avg_cnt)
{
    // Per LB5900 documentation Section IV - Time per Average based on MRATe setting:
    // NORMAL: 38.4ms, DOUBLE: 19.6ms, FAST: 3.2ms (forced to 1 avg), SUPER: 1.6ms
    uint32_t time_per_avg_us;
    switch(rate) {
        case 0:  // NORMAL
            time_per_avg_us = 38400;
            break;
        case 1:  // DOUBLE
            time_per_avg_us = 19600;
            break;
        case 2:  // FAST (averaging forced to 1)
            time_per_avg_us = 3200;
            avg_cnt = 1;
            break;
        case 3:  // SUPER
            time_per_avg_us = 1600;
            break;
        default:
            time_per_avg_us = 38400;  // Default to NORMAL
            break;
    }

    // Total measurement time = time_per_average * number_of_averages
    return time_per_avg_us * avg_cnt;
}

bool AP_ARRC_LB5900::_fetch_and_read(float &power)
{
    // Step 1: Send FETCh? command with header 0x06 (nextReadIsStatusAndLength)
    // This requests the current measurement from the sensor's circular buffer
    if (!_send_command("FETCh?")) {
        return false;
    }

    // Step 2: Read 4 bytes status and length
    uint8_t status[4] {};
    if (!_dev->transfer(nullptr, 0, status, sizeof(status))) {
        return false;
    }
    // status bit 4 (0x10): output available; then the length big-endian
    const uint32_t len = (uint32_t(status[1]) << 16) | (uint32_t(status[2]) << 8) | status[3];
    if (!(status[0] & 0x10) || len == 0) {
        return false;
    }
    // The length comes from the sensor: reject anything that would not fit the read buffer and its terminator
    if (len >= sizeof(read_sensor_buffer.byte)) {
        return false;
    }

    // Step 3: Request the complete output buffer using header 0x0C
    uint8_t request[4] {(uint8_t)nextReadIsCompleteOutputBuffer, uint8_t(len >> 16), uint8_t(len >> 8), uint8_t(len)};
    if (!_dev->transfer(request, sizeof(request), nullptr, 0)) {
        return false;
    }

    // Step 4: Read the measurement data
    memset(read_sensor_buffer.byte, 0x00, sizeof(read_sensor_buffer.byte));
    if (!_dev->transfer(nullptr, 0, read_sensor_buffer.byte, len)) {
        return false;
    }

    // Convert ASCII response to float (e.g., "-6.38395844E+01" -> -63.8395844)
    read_sensor_buffer.byte[len] = 0;
    char *end = nullptr;
    const float value = strtof(read_sensor_buffer.string, &end);
    if (end == read_sensor_buffer.string || !isfinite(value) || value < MIN_DBM || value > MAX_DBM) {
        return false;
    }
    power = value;
    return true;
}

// I2C bus thread
void AP_ARRC_LB5900::_timer(void)
{
    if (!_enabled) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();

    if (_state == State::CONFIGURING) {
        if (_config_attempt_ms != 0 && now_ms - _config_attempt_ms < CONFIG_RETRY_MS) {
            return;
        }
        _config_attempt_ms = now_ms;
        if (_configure()) {
            _last_ok_ms = now_ms;
            _ever_configured = true;
            _state = State::RUNNING;
        }
        return;
    }
    if (_state != State::RUNNING) {
        return;
    }

    // In free-run mode (INIT:CONT ON), the sensor continuously averages measurements.
    // FETCh? returns the most recent trailing average from the circular buffer.
    float power;
    if (_fetch_and_read(power)) {
        _last_ok_ms = now_ms;
        WITH_SEMAPHORE(_sem);
        _sample.power_dbm = power;
        _sample.count++;
        _sample.time_ms = now_ms;
        _have_sample = true;
    } else if (now_ms - _last_ok_ms > MAX(RECONFIGURE_MS, 8U * _period_ms)) {
        // no reading for a while: the sensor may have reset or been unplugged and plugged in again
        _config_attempt_ms = 0;
        _state = State::CONFIGURING;
    }
}

#endif  // AP_ARRC_LB5900_ENABLED
