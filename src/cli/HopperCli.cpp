#include "HopperCli.hpp"
#include "devices.hpp"
#include "drivers/serial/ISerial.hpp"
#include "utilities/print/Print.hpp"
#include "hopper_shared.hpp"
#include "WifiCli.hpp"
#include <string.h>

#include "Storage.hpp"

using namespace Cli;
using namespace SerialComm;
using namespace Dio;
using namespace Hopper;

extern ISerial* pUart;

const static uint8_t MAX_RESP_LEN = 12;
static char respBuffer[MAX_RESP_LEN + 1] = {0};
const static uint16_t RESP_TIMEOUT_MS = 60000;

static void statusCmd(uint16_t argc, ArgV argv)
{
    PRINTLN("Hopper is %d", pDoor->getState());
    pDoor->test();
}

static void openCmd(uint16_t argc, ArgV argv)
{
    pDoor->open(Commander::LOCAL);
}

static void closeCmd(uint16_t argc, ArgV argv)
{
    pDoor->close(Commander::LOCAL);
}

static void calibrationCmd(uint16_t argc, ArgV argv)
{
    pDoor->runCalibration(pUart, pWdt);
}

static void echoCmd(uint16_t argc, ArgV argv)
{
    pWifiCli->sendWifiCommand(ECHO_CMD_STR);
    memset(respBuffer, 0, sizeof(respBuffer));
    uint8_t respLen = pWifiSerial->readLine((uint8_t*)respBuffer, MAX_RESP_LEN, RESP_TIMEOUT_MS);
    respBuffer[respLen] = '\0';
    PRINTLN("R: %s", respBuffer);
}

static void idCmd(uint16_t argc, ArgV argv)
{
    if (argc >= 2)
    {
        pLed->set(Level::L_LOW);
        pWifiCli->sendWifiCommand(ID_CMD_STR, argv[1]);

        memset(respBuffer, 0, sizeof(respBuffer));
        uint8_t respLen = pWifiSerial->readLine((uint8_t*)respBuffer, MAX_RESP_LEN, RESP_TIMEOUT_MS);
        if ((respLen > 0) && (strncmp(respBuffer, PASS_STR, sizeof(PASS_STR)-1) == 0))
        {
            PRINTLN(PASS_STR);
        }
        else
        {
            PRINTLN(FAIL_STR);
        }
    }
}

static void wifiCmd(uint16_t argc, ArgV argv)
{
    if (argc  == 2)
    {
        pWifiCli->sendWifiCommand(WIFI_CMD_STR, argv[1]);
    }
    else if (argc == 3)
    {
        pWifiCli->sendWifiCommand(WIFI_CMD_STR, (char**)&(argv[1]), 2);
    }
    else
    {
        return;
    }

    pLed->set(Level::L_LOW);
    memset(respBuffer, 0, sizeof(respBuffer));
    uint8_t respLen = pWifiSerial->readLine((uint8_t*)respBuffer, MAX_RESP_LEN, RESP_TIMEOUT_MS);
    if ((respLen > 0) && (strncmp(respBuffer, PASS_STR, sizeof(PASS_STR)-1) == 0))
    {
        PRINTLN(PASS_STR);
    }
    else
    {
        PRINTLN(FAIL_STR);
    }
}

static void wifiStatusCmd(uint16_t argc, ArgV argv)
{
    pWifiCli->sendWifiCommand(MAIN_STATUS_CMD_STR);
}

const static Command commands[] =
{
    {.name = "STATUS", .function = &statusCmd},
    {.name = "OPEN", .function = &openCmd},
    {.name = "CLOSE", .function = &closeCmd},
    {.name = "CAL", .function = &calibrationCmd},
    {.name = ECHO_CMD_STR, .function = &echoCmd},
    {.name = ID_CMD_STR, .function = &idCmd},
    {.name = WIFI_CMD_STR, .function = &wifiCmd},
    {.name = "WS", .function = &wifiStatusCmd},
};
const static uint8_t NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);

static CommandInterface hopperCli(pUart, commands, NUM_COMMANDS);
CommandInterface* pCli = &hopperCli;