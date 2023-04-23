#include "DoorController.hpp"
#include "drivers/timer/Delay.hpp"
#include "utilities/print/Print.hpp"
#include "config.hpp"
#include "hopper_shared.hpp"
#include "cli/WifiCli.hpp"
#include "utilities/Conversions.hpp"

using namespace Servo;
using namespace Adc;
using namespace Dio;

namespace Hopper
{
    const static uint16_t CAL_DELAY_MS = 500;

    float DoorController::adcToAngle(
        uint16_t adcAtMin,
        uint16_t adcAtMax,
        uint16_t adcAngle)
    {
        float numerator = 0;
        // Ensure we do not go negative
        if (adcAngle > adcAtMin)
        {
            numerator = (adcAngle - adcAtMin) * (pServo_->getMaxAngle() - pServo_->getMinAngle());
        }

        float adcRange = (adcAtMax - adcAtMin);
        return (numerator / adcRange) + pServo_->getMinAngle(); 
    }

    DoorController::DoorController(Servo::IServo* pServo,
                           Adc::IAdc* pFeedback,
                           Tic::TicCounter* pTicHandler):
        pServo_(pServo),
        pFeedback_(pFeedback),
        timeoutTimer_(pTicHandler->secondsToTics(DOOR_TIMEOUT_S), pTicHandler),
        updateTimer_(SERVO_FEEDBACK_UPDATE_TICS, pTicHandler),
        angleFilter_(10, 0)
    {
        currAngle_ = 0;
        isRunning_ = false;
        currState_ = DOOR_AJAR;

        pFeedback_->enable();
        pServo_->disable();
        updateTimer_.enable();
    }

    bool DoorController::isCalibrated()
    {
        return pCalibration_->adcMax != pCalibration_->adcMin;
    }

    void DoorController::init()
    {
        // Find the starting angle of the servo, and set it to that angle
        // to avoid sudden movements during startup
        findState();
        pServo_->setAngle(currAngle_);
    }

    void DoorController::command(Commander commander,
                                 float targetAngle,
                                 bool requireCal)
    {
        if ((uint8_t)commander >= (uint8_t)Commander::NUM_COMMANDERS)
        {
            handleCommandComplete(false);
            return;
        }

        if (targetAngle < pServo_->getMinAngle()) targetAngle = pServo_->getMinAngle();
        if (targetAngle > pServo_->getMaxAngle()) targetAngle = pServo_->getMaxAngle();

        // To keeps the servo from immediatelly running to whatever angle
        // it was last set to, measure the current angle and pre-emptively
        // set to the servo to it. Now, when it turns on, it should jump
        if (!isRunning_)
        {
            findAngle();
            pServo_->setAngle(currAngle_);
        }

        targetAngle_ = targetAngle;
        speed_ = DOOR_ANG_VEL;
        commander_ = commander;
        isRunning_ = true;
        timeoutTimer_.enable();
        pServo_->enable();
    }

    void DoorController::open(Commander commander)
    {
        if (!isCalibrated())
        {
            PRINTLN("NOT CALIBRATED");
            handleCommandComplete(false);
            return;
        }

        command(commander, pCalibration_->angleOpen);
    }

    void DoorController::close(Commander commander)
    {
        if (!isCalibrated())
        {
            PRINTLN("NOT CALIBRATED");
            handleCommandComplete(false);
            return;
        }

        command(commander, pCalibration_->angleClosed);
    }

    void DoorController::toggle()
    {
        if (!isCalibrated())
        {
            PRINTLN("NOT CALIBRATED");
            handleCommandComplete(false);
            return;
        }

        findState();
        if (currState_ == DOOR_CLOSED)
        {
            command(Commander::LOCAL, pCalibration_->angleOpen);
        }
        else
        {
            command(Commander::LOCAL, pCalibration_->angleClosed);
        }
    }

    HopperState DoorController::getState()
    {
        if (isRunning_) return HopperState::DOOR_AJAR;
        findState();
        return currState_;
    }

    int16_t DoorController::readPosition()
    {
        bool success = false;
        uint32_t sum = 0;
        uint8_t numSamples = 0;
        for (uint8_t i=0; i<NUM_POS_SAMPLES; i++)
        {
            int16_t feedbackReading = pFeedback_->read(&success);;
            if (success)
            {
                sum += feedbackReading;
                numSamples++;
            }
            DELAY(SAMPLE_DELAY_MS);
        }

        if (numSamples == 0) return -1; // All reads failed

        return sum/numSamples;
    }

    float DoorController::findAngle()
    {
        if (!isCalibrated())
        {
            // Not calibrated
            return 0;
        }

        uint16_t position = readPosition();
        if (position < 0) return currAngle_;

        currAngle_ = adcToAngle(pCalibration_->adcMin, pCalibration_->adcMax, position);
        return currAngle_;
    }

    void DoorController::findState()
    {
        if (!isCalibrated())
        {
            currState_ = DOOR_UNCALIBRATED;
            return;
        }

        findAngle();
        float diffOpen = abs(currAngle_ - pCalibration_->angleOpen);
        float diffClosed = abs(currAngle_ - pCalibration_->angleClosed);

        if ((diffOpen > DOOR_ANGLE_LEEWAY) && (diffClosed > DOOR_ANGLE_LEEWAY))
        {
            currState_ = DOOR_AJAR;
        }
        else if (diffClosed < diffOpen)
        {
            currState_ = DOOR_CLOSED;
        }
        else
        {
            currState_ = DOOR_OPEN;
        }
    }

    void DoorController::update()
    {
        if (updateTimer_.hasPeriodPassed())
        {
            if (isRunning_)
            {
                // Movement time out
                if (timeoutTimer_.hasOneShotPassed())
                {
                    // Disable servo to allow for position to be read
                    pServo_->disable();

                    // Allow for the reading to settle
                    DELAY(100);
                    findState();

#ifdef DEBUG
                    PRINTLN("A: %f", currAngle_);
                    PRINTLN("State: %d", currState_);
#endif
                    handleCommandComplete(true);
                }
                else
                {
                    // Continue movement
                    int16_t servoAngle = pServo_->getAngle();
                    int16_t nextAngle = servoAngle;

                    if (targetAngle_ < servoAngle)
                    {
                        nextAngle = ((servoAngle - targetAngle_) >= speed_) ?
                                    servoAngle - speed_ :
                                    targetAngle_;
                    }
                    else if (targetAngle_ > servoAngle)
                    {
                        nextAngle = ((targetAngle_ - servoAngle) >= speed_) ?
                                    servoAngle + speed_ :
                                    targetAngle_;  
                    }
                    pServo_->setAngle(nextAngle);
                }
            }
        }
    }

    void DoorController::handleCommandComplete(bool success)
    {
        if (commander_ == Commander::REMOTE)
        {
            const char* resStr = success ? PASS_STR : FAIL_STR;
            char statusStr[2] =
            {
                currState_ + 0x30,
                '\0'
            };
            pWifiCli->sendWifiCommand(resStr, statusStr);
        }

        isRunning_ = false;
        timeoutTimer_.disable();
    }

    bool DoorController::storeCalMinAdc()
    {
        int16_t adcMin = readPosition();
        if (adcMin < 0)
        {
            PRINTLN("Could not read min ADC");
            return false;
        }
        pCalibration_->adcMin = (uint16_t)adcMin;
        PRINTLN("ADC min: %d", pCalibration_->adcMin);
        return true;
    }

    bool DoorController::storeCalMaxAdc()
    {
        int16_t adcMax = readPosition();
        if (adcMax < 0)
        {
            PRINTLN("Could not read max ADC");
            return false;
        }
        pCalibration_->adcMax = (uint16_t)adcMax;
        PRINTLN("ADC max: %d", pCalibration_->adcMax);
        return true;
    }

    bool DoorController::storeCalAngleOpen()
    {
        int16_t adcOpen = readPosition();
        if (adcOpen < 0)
        {
            PRINTLN("Could not read open ADC");
            return false;
        }
        PRINTLN("ADC Open: %d", adcOpen);
        pCalibration_->angleOpen = static_cast<int16_t>(adcToAngle(pCalibration_->adcMin, pCalibration_->adcMax, (uint16_t)adcOpen));
        PRINTLN("Angle Open: %d", pCalibration_->angleOpen);
        return true;
    }

    bool DoorController::storeCalAngleClosed()
    {
        int16_t adcClosed = readPosition();
        if (adcClosed < 0)
        {
            PRINTLN("Could not read closed ADC");
            return false;
        }
        PRINTLN("ADC Closed: %d", adcClosed);
        pCalibration_->angleClosed = static_cast<int16_t>(adcToAngle(pCalibration_->adcMin, pCalibration_->adcMax, (uint16_t)adcClosed));
        PRINTLN("Angle Closed: %d", pCalibration_->angleClosed);
        return true;
    }

    void DoorController::test()
    {
    }
}