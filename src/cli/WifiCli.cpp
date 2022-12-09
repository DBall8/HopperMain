#include "WifiCli.hpp"
#include "HopperCli.hpp"
#include "devices.hpp"
#include "utilities/print/Print.hpp"
#include "hopper_shared.hpp"
#include "utilities/strings/Strings.hpp"
#include "drivers/timer/Delay.hpp"

using namespace Cli;
using namespace SerialComm;
using namespace Strings;
using namespace Hopper;

extern ISerial* pUart;
static bool isWifiConn = false;

const static char NEWLINE = '\n';
const static char SPACE = ' ';

static void statusCmd(uint16_t argc, ArgV argv)
{
    // Convert numerical status to a ascii character
    char* statusStr = "3\0";
    statusStr[0] = pDoor->getState() + 0x30;
    pWifiCli->sendWifiCommand(statusStr);
}

static void openCmd(uint16_t argc, ArgV argv)
{
    pDoor->open(Commander::REMOTE);
}

static void closeCmd(uint16_t argc, ArgV argv)
{
    pDoor->close(Commander::REMOTE);
}

static void connectedCmd(uint16_t argc, ArgV argv)
{
    isWifiConn = true;
}

static void disconnectedCmd(uint16_t argc, ArgV argv)
{
    isWifiConn = false;
}

const static Command commands[] =
{
    {.name = STATUS_CMD, .function = &statusCmd},
    {.name = OPEN_CMD, .function = &openCmd},
    {.name = CLOSE_CMD, .function = &closeCmd},
    {.name = CONNECTED_STR, .function = &connectedCmd},
    {.name = DISCONNECTED_STR, .function = &disconnectedCmd},
};
const static uint8_t NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);

WifiCli::WifiCli(SerialComm::ISerial* pUart,
                const Command* commands,
                uint16_t numCommands,
                bool quite):
    CommandInterface(pUart, commands, numCommands, quite)
{
    
}

bool WifiCli::isConnected()
{
    return isWifiConn;
}

void WifiCli::sendWifiCommand(const char* cmd)
{
    sendWifiCommand(cmd, nullptr, 0);
}

void WifiCli::sendWifiCommand(const char* cmd, const char* value)
{
    sendWifiCommand(cmd, &value, (uint8_t)1);
}

void WifiCli::sendWifiCommand(const char* cmd, const char** values, uint8_t numVals)
{
    if (cmd == nullptr) return;

    if (!initialized_)
    {
        pWifiSerial->write(&NEWLINE, 1);
        DELAY(500);
        initialized_ = true;
    }

    pWifiSerial->flush();
    pWifiSerial->write(cmd, strlen(cmd));
    for (uint8_t i=0; i<numVals; i++)
    {
        pWifiSerial->write(&SPACE, 1);
        pWifiSerial->write(values[i], strlen(values[i]));
    }
    pWifiSerial->write(&NEWLINE, 1);
}

static WifiCli wifiCli(pWifiSerial, commands, NUM_COMMANDS, true);
WifiCli* pWifiCli = &wifiCli;