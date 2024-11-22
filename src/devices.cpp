#include "devices.hpp"
#include "config.hpp"

#include "drivers/watchdog/atmega328/Atmega328Watchdog.hpp"
#include "drivers/interrupt/atmega328/Atmega328Interrupt.hpp"
#include "drivers/timer/TicCounter.hpp"
#include "drivers/timer/Delay.hpp"
#include "drivers/timer/ATmega328/ATmega328Timer.hpp"
#include "drivers/dio/atmega328/Atmega328Dio.hpp"
#include "drivers/adc/atmega328/Atmega328Adc.hpp"
#include "drivers/serial/atmega328/Atmega328AsynchUart.hpp"
#include "drivers/button/digitalButton/DigitalButton.hpp"
#include "drivers/servo/IServo.hpp"
#include "drivers/eeprom/atmega328/Atmega328Eeprom.hpp"
#include "drivers/serial/atmega328/Atmega328SoftwareSerial.hpp"

#include "utilities/print/Print.hpp"

#include "Storage.hpp"
#include "cli/WifiCli.hpp"

using namespace Watchdog;
using namespace Timer;
using namespace Tic;
using namespace Interrupt;
using namespace SerialComm;
using namespace Dio;
using namespace Adc;
using namespace Button;
using namespace Servo;
using namespace Eeprom;
using namespace Hopper;

const static uint16_t WDT_TIMEOUT = 2000;
static Atmega328Watchdog wdt;
IWatchdog* pWdt = &wdt;

static Atmega328Interrupt intControl;
IInterrupt* pIntControl;

// Set up tic handler, make sure to attach this interrupt during init
static TicCounter ticHandler(TICS_PER_SECOND);
void HandleTicInterrupt()
{
    ticHandler.incrementTicCount();
}
TicCounter* pTicHandler = &ticHandler;

// Set up timer that triggers the tic counter to count
// 61 Hz = 16 MHz / (1024 * (255 + 1))
const static TimerPrescaler PRESCALE = PRESCALE_1024;
const static uint16_t TOP = 255;
static Atmega328Timer ticTmr(Timer::TIMER_2, CTC, PRESCALE, TOP, &HandleTicInterrupt);


// Construct main debug serial port
const static uint8_t RX_BUFF_SIZE = 32;
const static uint8_t TX_BUFF_SIZE = 64;
static uint8_t rxBuffer[RX_BUFF_SIZE];
static uint8_t txBuffer[TX_BUFF_SIZE];
static Atmega328AsynchUart uart(txBuffer,
                                rxBuffer,
                                TX_BUFF_SIZE,
                                RX_BUFF_SIZE,
                                BAUD_9600,
                                F_CPU,
                                &intControl);
ISerial* pUart = &uart;

// GPIOs
static Atmega328Dio buttonDio(Port::D, 2, Mode::INPUT, Level::L_LOW, false, false);
static Atmega328Dio servoSig(Port::D, 3, Mode::OUTPUT, Level::L_LOW, false, false);
// static Atmega328Dio wifiTx(Port::D, 6, Mode::INPUT, Level::L_LOW, false, false);
static Atmega328Dio wifiTx(Port::D, 6, Mode::OUTPUT, Level::L_LOW, false, false);
static Atmega328Dio wifiRx(Port::D, 7, Mode::INPUT, Level::L_LOW, false, true);
static Atmega328Dio ledPin(Port::D, 5, Mode::OUTPUT, Level::L_HIGH, false, false);
static Atmega328Dio wifiReset(Port::B, 0, Mode::OUTPUT, Level::L_HIGH, false, false);
static Atmega328Dio wifiProgram(Port::B, 1, Mode::OUTPUT, Level::L_HIGH, false, false);

IDio* pLed = &ledPin;

// ADCs
static Atmega328Adc servoFeedbackAdc(Atmega328Channel::ADC_0, Prescaler::DIV_128, Reference::AREF);

// Buttons
const static uint16_t DEBOUNCE_MS = 500;
static SoftwareTimer debounceTimer(ticHandler.msecondsToTics(DEBOUNCE_MS), &ticHandler);
static DigitalButton button(&buttonDio, &debounceTimer);
IButton* pToggleButton = &button;

// Servo
// Timer is 1ms - 2ms | FREQ = F_CPU / (PRESCALE * (TOP+1))
static Atmega328Timer servoTimer(Timer::TIMER_1,
                                TimerMode::NORMAL,
                                TimerPrescaler::PRESCALE_8,
                                0xffff,
                                nullptr);

static IServo servo(&servoTimer, &servoSig, MIN_SERVO_MICRO_S, MAX_SERVO_MICRO_S, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

static DoorController doorController(
    &servo,
    &servoFeedbackAdc,
    &ticHandler);
DoorController* pDoor = &doorController;

static Atmega328Eeprom eeprom(&intControl);
static EepromManager eepromManager(&eeprom, pStorageItems, sizeof(StorageItems));
EepromManager* pStorage = &eepromManager;

const static uint16_t WIFI_SERIAL_RX_BUFFER_LEN = 64;
static uint8_t wifiSerialRxBuffer[WIFI_SERIAL_RX_BUFFER_LEN];
static SoftwareTimer wifiSerialTimeoutTimer(0, pTicHandler, pWdt);
static Atmega328SoftwareSerial wifiSerial(&wifiRx, &wifiTx, &intControl, 9600, F_CPU, wifiSerialRxBuffer, WIFI_SERIAL_RX_BUFFER_LEN, &wifiSerialTimeoutTimer);
ISerial* pWifiSerial = &wifiSerial;

void initDevices()
{
    // Start watchdog
    wdt.init();
    wdt.setTimeout(WDT_TIMEOUT);
    wdt.enable();

    // Setup delay utility early, as some drivers make use of it
    Delay::Initialize(&ticHandler, &wdt);

    // Enable interrupts
    intControl.enableInterrupts();
    
    // Start the tic timer
    ticTmr.initialize();
    servoTimer.initialize();

    ticTmr.enable();

    // Set up serial
    uart.initialize();
    PrintHandler::getInstance().initialize(&uart);

    pWifiSerial->initialize();

    // Check for a reset cause
    ResetCause resetCause = pWdt->getResetCause();
    if (resetCause == ResetCause::WATCHDOG)
    {
        PRINTLN("!WDT!");
        pWifiCli->logBackend("WDT_R");
    }
    else if (resetCause == ResetCause::BROWN_OUT)
    {
        PRINTLN("!BO!");
        pWifiCli->logBackend("WDT_BO");
    }

    pStorage->initialize();

    if (pStorageItems->revision != STORAGE_REVISION)
    {
        // Reset to default values
        pStorageItems->calibration.adcMin = 0;
        pStorageItems->calibration.adcMax = 0;
        pStorageItems->calibration.angleClosed = 0;
        pStorageItems->calibration.angleOpen = 0;
        pStorageItems->revision = STORAGE_REVISION;
    }

    pDoor->setCalibration(&pStorageItems->calibration);
}

void checkErrors()
{
    if (wifiSerial.checkRxOverflow())
    {
        PRINTLN("!RX OVF!");
    }
}

void setEsp8Program(bool enabled)
{
    if (enabled)
    {
        wifiTx.setInputMode(false);

        // Pin dance to enter flash mode
        wifiProgram.set(L_LOW);
        DELAY(500);
        wifiReset.set(L_LOW);
        DELAY(500);
        wifiReset.set(L_HIGH);
        DELAY(500);
        wifiProgram.set(L_HIGH);
        PRINTLN(">P");
    }
    else
    {
        wifiTx.setOutputMode(L_HIGH);

        // Reset esp
        wifiReset.set(L_LOW);
        DELAY(500);
        wifiReset.set(L_HIGH);
        PRINTLN(">R");
    }
}

static bool setupMode = false;
void setSetupMode(bool newSetupMode)
{
    setupMode = newSetupMode;
}

bool getSetupMode()
{
    return setupMode;
}

void debugTest()
{

}