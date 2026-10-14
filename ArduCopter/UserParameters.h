#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    // AP_Int8 get_int8Param() const { return _int8; }
    // AP_Int16 get_int16Param() const { return _int16; }
    // AP_Float get_floatParam() const { return _float; }

    // ARRC LB5900
    AP_Int8 get_lb5900_address() const{return lb5900_addr; }
    AP_Int16 get_lb5900_freq() const{return lb5900_freq; }
    AP_Int8 get_lb5900_avg_cnt() const{return lb5900_avg_cnt; }
    AP_Int8 get_lb5900_mrate() const{return lb5900_mrate; }
    AP_Int8 get_lb6900_busId() const{return lb5900_busId; }

    // ARRC onboard SDR
    int8_t get_rf_src() const { return rf_src; }
    int16_t get_sdr_options() const { return sdr_options; }
    float get_sdr_timeout() const { return sdr_timeout; }
    
private:
    // Put your parameter variable definitions here
    // AP_Int8 _int8;
    // AP_Int16 _int16;
    // AP_Float _float;

    //ARRC LB5900
    AP_Int8 lb5900_addr;
    AP_Int16 lb5900_freq;
    AP_Int8 lb5900_avg_cnt;
    AP_Int8 lb5900_mrate;
    AP_Int8 lb5900_busId;

    // ARRC onboard SDR
    AP_Int8 rf_src;
    AP_Int16 sdr_options;
    AP_Float sdr_timeout;
};
