#ifndef DOOR_CONTROLLER_HPP
#define DOOR_CONTROLLER_HPP

#include "drivers/servo/IServo.hpp"
#include "drivers/Dio/IDio.hpp"
#include "drivers/adc/IAdc.hpp"
#include "drivers/timer/SoftwareTimer.hpp"
#include "drivers/timer/TicCounter.hpp"
#include "drivers/watchdog/Watchdog.hpp"
#include "utilities/filter/lowPassFilter/LowPassFilter.hpp"
#include "drivers/serial/ISerial.hpp"

namespace Hopper
{
    enum HopperState : uint8_t
    {
        DOOR_OPEN = 0,
        DOOR_CLOSED,
        DOOR_AJAR,

        DOOR_INVALID
    };

    enum class Commander : uint8_t
    {
        LOCAL = 0,
        REMOTE,
        INTERNAL,

        NUM_COMMANDERS
    };

    struct HopperCalibration
    {
        uint16_t adcMin;
        uint16_t adcMax;
        int16_t angleClosed;
        int16_t angleOpen;
    };

    class DoorController
    {
        public:
            DoorController(Servo::IServo* pServo,
                           Adc::IAdc* pFeedback,
                           Tic::TicCounter* pTicHandler,
                           uint8_t angularVelocity,
                           Dio::IDio* pServoEn = nullptr);
            ~DoorController(){}

            void init();
            void open(Commander commander);
            void close(Commander commander);
            void toggle();
            HopperState getState();
            void runCalibration(SerialComm::ISerial* pSerial, Watchdog::IWatchdog* pWdt);
            void setCalibration(HopperCalibration* pCalibration){ pCalibration_ = pCalibration; }

            void update();

            void test();

        private:
            Servo::IServo* pServo_;
            Adc::IAdc* pFeedback_;
            uint8_t angularVelocity_;
            Dio::IDio* pServoEn_;
            Timer::SoftwareTimer timeoutTimer_;
            Timer::SoftwareTimer powerTimer_;
            Timer::SoftwareTimer updateTimer_;
            HopperCalibration* pCalibration_;

            HopperState currState_;
            HopperState targetState_;
            Commander currCommander_;
            Filter::LowPassFilter angleFilter_;
            float currAngle_;
            bool isRunning_;
            bool isPowered_;
            bool isReturning_;
            bool hasCommandFailed_;

            float getAngle();
            void findState();
            bool isCalibrated();

            void powerServo();

            void handleMovementComplete();

            int16_t readPos();
    };
}

#endif