// user defined variables
#include "APM_Config.h"
// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#define LOG_IMET_MSG 0x29 // SD Card Memory Direction for Temp sensor
#define LOG_RH_MSG 0x30 // SD Card Memory Direction for RH sensor
#define LOG_WIND_MSG 0x31 //SD Card Memory Direction for our Wind estimation

struct PACKED log_RH {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    float humidity1;
    float RHtemp1;
    float humidity2;
    float RHtemp2;
    float humidity3;
    float RHtemp3;
    float humidity4;
    float RHtemp4;
};

struct PACKED log_IMET {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    float temperature1;
    float voltage1;
    float temperature2;
    float voltage2;
    float temperature3;
    float voltage3;
    float temperature4;
    float voltage4;
    uint8_t fan_status;
};

struct PACKED log_WIND {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    float _wind_dir;
    float _wind_speed;
    float _wind_dir_var;
    float _gamma_var;
    float _roll_sum;
    float _pitch_sum;
    float _yaw;
};

float cass_wind_direction;
float cass_wind_speed;

#endif  // USERHOOK_VARIABLES



