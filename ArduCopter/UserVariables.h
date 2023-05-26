// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#define LOG_LB680A_MSG 0x32 //SD Card Memory Direction for our LadyBug 680A

struct PACKED log_LB680A {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    float pwr;
    float pkpwr;
    float avgpwr;
    float dcyc;
};

#endif  // USERHOOK_VARIABLES


