#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// ARRC onboard SDR (companion computer) interface. On this branch Copter runs with it by default; built
// without it (AP_ARRC_SDR_ENABLED=0) the vehicle keeps the LB5900 and the gimbal pointing only
#ifndef AP_ARRC_SDR_ENABLED
#define AP_ARRC_SDR_ENABLED 1
#endif
