// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#define LOG_LB5900_MSG 0x32 //SD Card Memory Direction for our LB5900

struct PACKED log_LB5900 {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    uint8_t healthy;
    float power;
};

#endif  // USERHOOK_VARIABLES


