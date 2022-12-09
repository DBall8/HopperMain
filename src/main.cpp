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

extern "C" void __cxa_pure_virtual() { while (1); }

void setup();
void loop();

static SoftwareTimer ticTimer(1, pTicHandler, pWdt);
static SoftwareTimer flashTimer(1, pTicHandler);

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
    PRINTLN("Hopper [%d] started.", HOPPER_ID);
    PRINTLN("----------");

    // In case we rebooted but the wifi board did not, get the current wifi status
    pWifiCli->sendWifiCommand(MAIN_STATUS_CMD_STR);
}

void updateLed();

void loop()
{
    pWdt->reset();

    pToggleButton->update();
    if (pToggleButton->getTransition() == ButtonTransition::PRESSED)
    {
#ifdef DEBUG
        PRINTLN("BUTTON PRESSED");
#endif
        pDoor->toggle();
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
    if (isWifiConn != isWifiConnNew)
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
    }

    if (!isWifiConn && flashTimer.hasPeriodPassed())
    {
        pLed->toggle();
        flashTimer.enable();
    }
}