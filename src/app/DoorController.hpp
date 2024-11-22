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
        DOOR_UNCALIBRATED = 0,
        DOOR_OPEN,
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

    enum class CalibrationStep : uint8_t
    {
        UNDEFINED = 0,
        START,
        WAIT_FOR_MIN,
        WAIT_FOR_MAX,
        WAIT_FOR_CLOSE,
        WAIT_FOR_OPEN,
        SUCCESS,
        FAIL
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
                           Tic::TicCounter* pTicHandler);
            ~DoorController(){}

            void init();
            void command(Commander commander, float targetAngle, bool requireCal = true);
            void open(Commander commander);
            void close(Commander commander);
            void toggle();
            HopperState getState();
            void wiggle(Commander commander);
            bool wiggleCustom(uint16_t angle, uint16_t stepMs);

            CalibrationStep getCalibrationStep(){ return currCalibrationStep_; }
            void startCalibration();
            void storeCalAngleOpen();
            void storeCalAngleClosed();
            bool setCalibration(HopperCalibration* pCalibration);

            bool isCommandInProgress(){ return isRunning_; }
            void update();

            void test();

        private:
            Servo::IServo* pServo_;
            Adc::IAdc* pFeedback_;
            Timer::SoftwareTimer timeoutTimer_;
            Timer::SoftwareTimer updateTimer_;
            Timer::SoftwareTimer calibrationTimer_;
            Timer::SoftwareTimer wiggleTimer_;
            HopperCalibration* pCalibration_;
            Filter::LowPassFilter angleFilter_;

            HopperState currState_;
            CalibrationStep currCalibrationStep_;
            float currAngle_;

            bool isRunning_;
            float targetAngle_;
            float speed_;
            Commander commander_;

            float adcToAngle(uint16_t adcAtMin, uint16_t adcAtMax, uint16_t adcAngle);
            int16_t readPosition();
            float findAngle();
            void findState();
            bool isCalibrated();

            void calibrationUpdate();
            bool storeCalMinAdc();
            bool storeCalMaxAdc();

            void handleCommandComplete(bool success);
    };
}

#endif