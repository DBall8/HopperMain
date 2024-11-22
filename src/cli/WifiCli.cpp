#include "WifiCli.hpp"
#include "HopperCli.hpp"
#include "devices.hpp"
#include "utilities/print/Print.hpp"
#include "hopper_shared.hpp"
#include "utilities/strings/Strings.hpp"
#include "drivers/timer/Delay.hpp"

using namespace Cli;
using namespace SerialComm;
using namespace StringUtilities;
using namespace Hopper;

extern ISerial* pUart;
static bool isWifiConn = false;

const static char NEWLINE = '\n';
const static char SPACE = ' ';
const static char UNDERSCORE = '_';

const static char* CAL_START_STR = "S";
const static char* CAL_CLOSE_STR = "C";
const static char* CAL_OPEN_STR = "O";

static void statusCmd(uint16_t argc, ArgV argv)
{
    // Convert numerical status to a ascii character
    char statusStr[2] =
    {
        (char)(pDoor->getState() + 0x30),
        '\0'
    };
    
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

static void calibrateCmd(uint16_t argc, ArgV argv)
{
    if (argc < 2)
    {
        // Invalid input
        pWifiCli->sendWifiCommand(CAL_FAIL_STR);
        return;
    }

    if (strcompare(argv[1], CAL_START_STR))
    {
        pDoor->startCalibration();

        while ((pDoor->getCalibrationStep() != CalibrationStep::WAIT_FOR_CLOSE) &&
               (pDoor->getCalibrationStep() != CalibrationStep::FAIL))
        {
            pDoor->update();
            pWdt->reset();
        }

        pWifiCli->sendWifiCommand(
            (pDoor->getCalibrationStep() == CalibrationStep::WAIT_FOR_CLOSE) ?
                CAL_PASS_STR :
                CAL_FAIL_STR);
        return;
    }

    if (strcompare(argv[1], CAL_CLOSE_STR))
    {
        pDoor->storeCalAngleClosed();

        pWifiCli->sendWifiCommand(
            (pDoor->getCalibrationStep() == CalibrationStep::WAIT_FOR_OPEN) ?
                CAL_PASS_STR :
                CAL_FAIL_STR);
        return;
    }

    if (strcompare(argv[1], CAL_OPEN_STR))
    {
        pDoor->storeCalAngleOpen();

        pWifiCli->sendWifiCommand(
            (pDoor->getCalibrationStep() == CalibrationStep::SUCCESS) ?
                CAL_PASS_STR :
                CAL_FAIL_STR);
        return;
    }

    // Invalid input
    pWifiCli->sendWifiCommand(CAL_FAIL_STR);
    return;
}

static void wiggleCmd(uint16_t argc, ArgV argv)
{
    pDoor->wiggle(Commander::REMOTE);
}

static void connectedCmd(uint16_t argc, ArgV argv)
{
    isWifiConn = true;
    PRINTLN("Connected");
}

static void disconnectedCmd(uint16_t argc, ArgV argv)
{
    isWifiConn = false;
    PRINTLN("Disconnected");
}

static void setupCmd(uint16_t argc, ArgV argv)
{
    if (argc != 2)
    {
        return;
    }

    if (argv[1][0] == 't')
    {
        setSetupMode(true);
    }
    else if (argv[1][0] == 'f')
    {
        setSetupMode(false);
    }
}

static void logCmd(uint16_t argc, ArgV argv)
{
    PRINT("[");
    for (uint8_t i=1; i<argc; i++)
    {
        PRINT("%s", argv[i]);
        if ((uint8_t)(i+1) < argc)
        {
            PRINT(" ");
        }
    }
    PRINT("]\n");
}

const static Command commands[] =
{
    {.name = STATUS_CMD,        .function = &statusCmd},
    {.name = OPEN_CMD,          .function = &openCmd},
    {.name = CLOSE_CMD,         .function = &closeCmd},
    {.name = CALIBRATE_CMD,     .function = &calibrateCmd},
    {.name = WIGGLE_CMD,        .function = &wiggleCmd},
    {.name = CONNECTED_STR,     .function = &connectedCmd},
    {.name = DISCONNECTED_STR,  .function = &disconnectedCmd},
    {.name = SETUP_CMD,         .function = &setupCmd},
    {.name = LOG_CMD,           .function = &logCmd},
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

#ifdef DEBUG_BACKEND
void WifiCli::logBackend(const char* log)
{
    if (!initialized_)
    {
        pWifiSerial->write(&NEWLINE, 1);
        DELAY(500);
        initialized_ = true;
    }

    pWifiSerial->flush();
    pWifiSerial->write(LOG_CMD_STR, strlen(LOG_CMD_STR));
    pWifiSerial->write(&SPACE, 1);
    pWifiSerial->write(log, strlen(log));
    pWifiSerial->write(&NEWLINE, 1);
}

void WifiCli::logBackend(const char* log, uint16_t value)
{
    if (!initialized_)
    {
        pWifiSerial->write(&NEWLINE, 1);
        DELAY(500);
        initialized_ = true;
    }

    int2str(value, numValBuffer, NUM_VAL_BUFF_LEN);

    pWifiSerial->flush();
    pWifiSerial->write(LOG_CMD_STR, strlen(LOG_CMD_STR));
    pWifiSerial->write(&SPACE, 1);
    pWifiSerial->write(log, strlen(log));
    pWifiSerial->write(&UNDERSCORE, 1);
    pWifiSerial->write(numValBuffer, strlen(numValBuffer));
    pWifiSerial->write(&NEWLINE, 1);
}

#else
void WifiCli::logBackend(const char* log){ (void)log; }
void WifiCli::logBackend(const char* log, uint16_t value) {(void)log; (void)value; }
#endif