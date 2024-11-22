#ifndef HOPPER_SHARED_HPP
#define HOPPER_SHARED_HPP

// Wifi to Main messages
const static char PASS_STR[]            = "<PASS";
const static char FAIL_STR[]            = "<FAIL";
const static char ECHO_RESP_STR[]       = "<ECHO RESP";
const static char DISCONNECTED_STR[]    = "<DC";
const static char CONNECTED_STR[]       = "<CN";
const static char STATUS_CMD[]          = "<STATUS";
const static char OPEN_CMD[]            = "<OPEN";
const static char CLOSE_CMD[]           = "<CLOSE";
const static char CALIBRATE_CMD[]       = "<CAL";
const static char LOG_CMD[]             = "<L";
const static char SETUP_CMD[]           = "<I";
const static char WIGGLE_CMD[]          = "<WIG";

// Main to Wifi commands
const static char ECHO_CMD_STR[]        = "ECHO";
const static char WIFI_CMD_STR[]        = "WIFI";
const static char ID_CMD_STR[]          = "ID";
const static char CLEAR_CMD_STR[]       = "CLEAR";
const static char GET_CMD_STR[]         = "L";
const static char LOG_CMD_STR[]         = "LOG";
const static char SETUP_CMD_STR[]       = "SETUP";
const static char CAL_PASS_STR[]        = "CAL0";
const static char CAL_FAIL_STR[]        = "CAL1";

#define MAIN_STATUS_CMD_STR  (STATUS_CMD+1)

#endif