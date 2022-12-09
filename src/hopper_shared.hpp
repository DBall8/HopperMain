#ifndef HOPPER_SHARED_HPP
#define HOPPER_SHARED_HPP

// Wifi to Main messages
const static char PASS_STR[] = "<PASS";
const static char FAIL_STR[] = "<FAIL";
const static char* ECHO_RESP_STR = "<ECHO RESP";
const static char* DISCONNECTED_STR = "<DC";
const static char* CONNECTED_STR = "<CN";
const static char* STATUS_CMD = "<STATUS";
const static char* OPEN_CMD = "<OPEN";
const static char* CLOSE_CMD = "<CLOSE";

// Main to Wifi commands
const static char* ECHO_CMD_STR = "ECHO";
const static char* WIFI_CMD_STR = "WIFI";
const static char* ID_CMD_STR = "ID";
const static char* CLEAR_CMD_STR = "CLEAR";
const static char* MAIN_STATUS_CMD_STR = STATUS_CMD+1;

#endif