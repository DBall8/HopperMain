#include "devices.hpp"
#include "config.hpp"
#include "drivers/timer/SoftwareTimer.hpp"
#include "utilities/print/Print.hpp"
#include "cli/HopperCli.hpp"
#include "cli/WifiCli.hpp"
#include "drivers/timer/Delay.hpp"
#include "hopper_shared.hpp"

using namespace Timer;
using namespace Button;
using namespace Dio;
using namespace Hopper;

extern "C" void __cxa_pure_virtual() { while (1); }

void setup();
void loop();
void updateLed();
void runCalibration();

static SoftwareTimer ticTimer(1, pTicHandler, pWdt);
static SoftwareTimer flashTimer(1, pTicHandler);
static SoftwareTimer buttonHoldTimer(1, pTicHandler);
bool buttonHeld = false;
bool resetLed = false;

int main(void)
{
    setup();

    for (;;) {
        if (ticTimer.hasPeriodPassed())
        {
            loop();
        }
    }
  return 0;
}

void setup()
{
    initDevices();
    ticTimer.enable();
    pCli->enable();
    pWifiCli->enable();
    pDoor->init();

    flashTimer.setPeriod(pTicHandler->secondsToTics(1));
    flashTimer.enable();

    PRINTLN("----------");
    PRINTLN("Hopper started.");
#ifdef PROG_WIFI
    PRINTLN("WIFI SERIAL DISABLED");
#endif
    PRINTLN("----------");

    // In case we rebooted but the wifi board did not, get the current wifi status
    pWifiCli->sendWifiCommand(MAIN_STATUS_CMD_STR);

    buttonHoldTimer.setPeriod(pTicHandler->secondsToTics(5));
}

void loop()
{
    pWdt->reset();

    pToggleButton->update();
    if (pToggleButton->getTransition() == ButtonTransition::PRESSED)
    {
        buttonHoldTimer.enable();
    }
    else if (pToggleButton->getState() == ButtonState::PRESSED)
    {
        if (buttonHoldTimer.hasOneShotPassed() & !buttonHeld)
        {
            buttonHeld = true;
            runCalibration();
        }
    }
    else if (pToggleButton->getTransition() == ButtonTransition::RELEASED)
    {
        if (!buttonHeld)
        {
#ifdef DEBUG
            PRINTLN("BUTTON PRESSED");
#endif
            pDoor->toggle();
        }
        buttonHoldTimer.disable();
        buttonHeld = false;
    }

    updateLed();

    pDoor->update();
    pCli->update();
    pWifiCli->update();
    pStorage->update();
}

void updateLed()
{
    static bool isWifiConn = false;
    bool isWifiConnNew = pWifiCli->isConnected();
    if ((isWifiConn != isWifiConnNew) || resetLed)
    {
        if (isWifiConnNew)
        {
            pLed->set(Level::L_HIGH);
            flashTimer.disable();
        }
        else
        {
            flashTimer.enable();
        }

        isWifiConn = isWifiConnNew;
        resetLed = false;
    }

    if (!isWifiConn && flashTimer.hasPeriodPassed())
    {
        pLed->toggle();
        flashTimer.enable();
    }
}

void waitForButtonTransition(ButtonTransition transition)
{
    pToggleButton->update();
    while(pToggleButton->getTransition() != transition)
    {
        pWdt->reset();
        DELAY(100);
        pToggleButton->update();
    }
}

void runCalibration()
{
    PRINTLN("Running calibration....");
    resetLed = true;
    pLed->set(Level::L_LOW);

    // Find the ADC reading at the minimum angle
    pDoor->command(Commander::LOCAL, MIN_SERVO_ANGLE);
    while (pDoor->isCommandInProgress())
    {
        pDoor->update();
        pWdt->reset();
    }
    if (!pDoor->storeCalMinAdc()) return;

    // Find the ADC at the max angle
    pDoor->command(Commander::LOCAL, MAX_SERVO_ANGLE);
    while (pDoor->isCommandInProgress())
    {
        pDoor->update();
        pWdt->reset();
    }
    if (!pDoor->storeCalMaxAdc()) return;

    // Find the desired closed angle
    PRINTLN("Please close then press the button");
    pLed->set(Level::L_HIGH);
    waitForButtonTransition(ButtonTransition::PRESSED);
    pLed->set(Level::L_LOW);
    if (!pDoor->storeCalAngleClosed()) return;

    // Find the desired open angle
    PRINTLN("Please open then press the button");
    pLed->set(Level::L_HIGH);
    waitForButtonTransition(ButtonTransition::PRESSED);
    pLed->set(Level::L_LOW);
    if (!pDoor->storeCalAngleOpen()) return;

    PRINTLN("Complete!");
}