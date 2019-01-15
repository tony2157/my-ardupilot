// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

#define USERHOOK_VARIABLES "UserVariables.h"
#define USERHOOK_INIT userhook_init();                      // for code to be run once at startup
#define USERHOOK_CO2LOOP userhook_CO2();            
//#define USERHOOK_TEMPLOOP userhook_TempLoop();                       
#define USERHOOK_RHLOOP userhook_RHLoop();            


