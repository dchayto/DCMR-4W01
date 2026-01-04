//								cmd_flag.hpp
// bit field var for setting command states
static unsigned char CMD_FLAG { 0b00000000 };	

// bit field definitions (to be used with CMD_FLAG)
const unsigned char WHEELCMD_RECEIVED 	{ 0b00000001 };
const unsigned char ENCODER_DATA_READ	{ 0b00000010 };
