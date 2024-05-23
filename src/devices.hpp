#ifndef DEVICES_HPP
#define DEVICES_HPP

#include "drivers/watchdog/Watchdog.hpp"
#include "drivers/timer/TicCounter.hpp"
#include "drivers/dio/IDio.hpp"
#include "drivers/button/IButton.hpp"
#include "drivers/eeprom/EepromManager.hpp"
#include "app/DoorController.hpp"
#include "drivers/serial/ISerial.hpp"

extern Watchdog::IWatchdog* pWdt;
extern Tic::TicCounter* pTicHandler;
extern Dio::IDio* pLed;
extern SerialComm::ISerial* pWifiSerial;
extern SerialComm::ISerial* pUart;
extern Button::IButton* pToggleButton;
extern Eeprom::EepromManager* pStorage;
extern Hopper::DoorController* pDoor;

void initDevices();
void checkErrors();

void debugTest();

#endif