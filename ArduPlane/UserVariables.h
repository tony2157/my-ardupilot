// user defined variables
#include "APM_Config.h"
// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#define LOG_IMET_MSG 0x29 // SD Card Memory Direction for Temp sensor
#define LOG_RH_MSG 0x30 // SD Card Memory Direction for RH sensor
#define LOG_WIND_MSG 0x31 //SD Card Memory Direction for our Wind estimation
#define LOG_CO2_MSG 0x32 //SD Card Memory Direction for our CO2 value.

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
};

struct PACKED log_CO2 {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    uint16_t co2Value0;
    uint16_t errorCode0;
    uint16_t co2Value1;
    uint16_t errorCode1;
};

#endif  // USERHOOK_VARIABLES


