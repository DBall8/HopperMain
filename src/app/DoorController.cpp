#include "DoorController.hpp"
#include "drivers/timer/Delay.hpp"
#include "utilities/print/Print.hpp"
#include "config.hpp"
#include "hopper_shared.hpp"
#include "cli/WifiCli.hpp"

using namespace Servo;
using namespace Adc;
using namespace Dio;

namespace Hopper
{
    const static uint16_t POWER_TIMEOUT_MS = 15 * 1000;
    const static uint16_t CAL_DELAY_MS = 500;

    static float adcToAngle(
        uint16_t adcAtMin,
        uint16_t adcAtMax,
        uint16_t adcAngle)
    {
        float numerator = 0;
        // Ensure we do not go negative
        if (adcAngle > adcAtMin)
        {
            numerator = (adcAngle - adcAtMin) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
        }

        float adcRange = (adcAtMax - adcAtMin);
        return (numerator / adcRange) + SERVO_MIN_ANGLE; 
    }

    DoorController::DoorController(Servo::IServo* pServo,
                           Adc::IAdc* pFeedback,
                           Tic::TicCounter* pTicHandler,
                           uint8_t angularVelocity,
                           Dio::IDio* pServoEn):
        pServo_(pServo),
        pFeedback_(pFeedback),
        angularVelocity_(angularVelocity),
        pServoEn_(pServoEn),
        timeoutTimer_(pTicHandler->secondsToTics(DOOR_TIMEOUT_S), pTicHandler),
        powerTimer_(pTicHandler->msecondsToTics(POWER_TIMEOUT_MS), pTicHandler),
        updateTimer_(SERVO_FEEDBACK_UPDATE_TICS, pTicHandler),
        angleFilter_(10, 0)
    {
        currAngle_ = 0;
        isRunning_ = false;
        isReturning_ = false;
        isPowered_ = false;
        currState_ = DOOR_AJAR;
        targetState_ = DOOR_CLOSED;
        currCommander_ = Commander::INTERNAL;

        pFeedback_->enable();
        pServo_->disable();
        powerTimer_.enable();
        updateTimer_.enable();
    }

    bool DoorController::isCalibrated()
    {
        return (pCalibration_->adcMax != 0) &&
               (pCalibration_->angleOpen != 0);
    }

    void DoorController::init()
    {
        // Find the starting angle of the servo, and set it to that angle
        // to avoid sudden movements during startup
        findState();
        pServo_->setAngle(currAngle_);
    }

    void DoorController::powerServo()
    {
        if (pServoEn_ == nullptr) return;

        if (!isPowered_)
        {
            pServoEn_->set(Level::L_LOW);
            isPowered_ = true;
            DELAY(100);
        }
        powerTimer_.enable();
    }

    void DoorController::open(Commander commander)
    {
        if (isReturning_) return;
        if ((uint8_t)commander >= (uint8_t)Commander::NUM_COMMANDERS) return;

        if (!isRunning_)
        {
            findState();
            pServo_->setAngle(currAngle_);
        }

        if (isCalibrated())
        {
            if (commander != Commander::INTERNAL)
            {
                currCommander_ = commander;
                hasCommandFailed_ = false;
            }
            
            powerServo();
            targetState_ = HopperState::DOOR_OPEN;
            timeoutTimer_.enable();
            isRunning_ = true;
            pServo_->enable();
        }
        else
        {
            if (commander == Commander::REMOTE)
            {
                PRINTLN(FAIL_STR);
            }
            PRINTLN("NOT CALIBRATED");
        }
    }

    void DoorController::close(Commander commander)
    {
        if (isReturning_) return;
        if ((uint8_t)commander >= (uint8_t)Commander::NUM_COMMANDERS) return;

        if (!isRunning_)
        {
            findState();
            pServo_->setAngle(currAngle_);
        }

        if (isCalibrated())
        {
            if (commander != Commander::INTERNAL)
            {
                currCommander_ = commander;
                hasCommandFailed_ = false;
            }

            powerServo();
            targetState_ = HopperState::DOOR_CLOSED;
            timeoutTimer_.enable();
            isRunning_ = true;
            pServo_->enable();
        }
        else
        {
            if (commander == Commander::REMOTE)
            {
                PRINTLN(FAIL_STR);
            }
            PRINTLN("NOT CALIBRATED");
        }
    }

    void DoorController::toggle()
    {
        getAngle();
        bool closedAngleSmaller = (pCalibration_->angleClosed < pCalibration_->angleOpen);
        int16_t diffOpen = closedAngleSmaller ?
            (pCalibration_->angleOpen - currAngle_) :
            (currAngle_ - pCalibration_->angleOpen);
        int16_t diffClosed = closedAngleSmaller ?
            (currAngle_ - pCalibration_->angleClosed) :
            (pCalibration_->angleClosed - currAngle_);
        if (diffClosed < diffOpen)
        {
            open(Commander::LOCAL);
        }
        else
        {
            close(Commander::LOCAL);
        }
    }

    HopperState DoorController::getState()
    {
        if (isRunning_) return HopperState::DOOR_AJAR;
        findState();
        return currState_;
    }

    void DoorController::runCalibration(SerialComm::ISerial* pSerial, Watchdog::IWatchdog* pWdt)
    {
        bool success = false;
        powerServo();
        PRINTLN("Running calibration....");

        // Find the ADC reading at the minimum angle
        pServo_->setAngle(SERVO_MIN_ANGLE);
        pServo_->enable();
        DELAY(CAL_DELAY_MS);
        pServo_->disable();
        DELAY(CAL_DELAY_MS);
        int16_t adcMin = readPos();
        if (adcMin < 0)
        {
            PRINTLN("Could not read min ADC");
            return;
        }
        pCalibration_->adcMin = (uint16_t)adcMin;
        PRINTLN("ADC min: %d", pCalibration_->adcMin);

        // Find the ADC at the max angle
        pServo_->setAngle(SERVO_MAX_ANGLE);
        pServo_->enable();
        DELAY(CAL_DELAY_MS);
        pServo_->disable();
        DELAY(CAL_DELAY_MS);
        int16_t adcMax = readPos();
        if (adcMax < 0)
        {
            PRINTLN("Could not read max ADC");
            return;
        }
        pCalibration_->adcMax = (uint16_t)adcMax;
        PRINTLN("ADC max: %d", pCalibration_->adcMax);

        // Find the desired closed angle
        PRINTLN("Please close");
        pSerial->flushRx();
        while(!pSerial->isDataAvailable())
        {
            pWdt->reset();
        }
        int16_t adcClosed = readPos();
        if (adcClosed < 0)
        {
            PRINTLN("Could not read closed ADC");
            return;
        }
        PRINTLN("ADC Closed: %d", adcClosed);
        pCalibration_->angleClosed = static_cast<int16_t>(adcToAngle(adcMin, adcMax, (uint16_t)adcClosed));
        PRINTLN("Angle Closed: %d", pCalibration_->angleClosed);

        // Find the desired open angle
        PRINTLN("Please open");
        pSerial->flushRx();
        while(!pSerial->isDataAvailable())
        {
            pWdt->reset();
        }
        int16_t adcOpen = readPos();
        if (adcOpen < 0)
        {
            PRINTLN("Could not read open ADC");
            return;
        }
        PRINTLN("ADC Open: %d", adcOpen);
        pCalibration_->angleOpen = static_cast<int16_t>(adcToAngle(adcMin, adcMax, (uint16_t)adcOpen));
        pSerial->flushRx();
        PRINTLN("Angle Open: %d", pCalibration_->angleOpen);
        PRINTLN("Complete!");
    }

    float DoorController::getAngle()
    {
        if (!isCalibrated())
        {
            // Not calibrated
            return 0;
        }

        // Power is needed in order to read angle
        powerServo();

        int16_t feedbackReading = readPos();
        if (feedbackReading < 0)
        {
            // Failed to read, send last angle to avoid sudden movements
            return currAngle_;
        }

        //angleFilter_.addSample((int)feedbackReading);

        currAngle_ = adcToAngle(pCalibration_->adcMin, pCalibration_->adcMax, feedbackReading);
        return currAngle_;
    }

    void DoorController::findState()
    {
        getAngle();

        if (pCalibration_->angleClosed < pCalibration_->angleOpen)
        {
            // Closed is the lower angle
            if (currAngle_ <= (pCalibration_->angleClosed + DOOR_ANGLE_LEEWAY))
            {
                currState_ = HopperState::DOOR_CLOSED;
            }
            else if (currAngle_ >= (pCalibration_->angleOpen - DOOR_ANGLE_LEEWAY))
            {
                currState_ = HopperState::DOOR_OPEN;
            }
            else
            {
                currState_ = HopperState::DOOR_AJAR;
            }
       }
       else
       {
           // Open is the lower angle
            if (currAngle_ <= (pCalibration_->angleOpen + DOOR_ANGLE_LEEWAY))
            {
                currState_ = HopperState::DOOR_OPEN;
            }
            else if (currAngle_ >= (pCalibration_->angleClosed - DOOR_ANGLE_LEEWAY))
            {
                currState_ = HopperState::DOOR_CLOSED;
            }
            else
            {
                currState_ = HopperState::DOOR_AJAR;
            }
       }
    }

    void DoorController::update()
    {
        if (isPowered_ &&
            (pServoEn_ != nullptr) &&
            powerTimer_.isEnabled() &&
            powerTimer_.hasPeriodPassed())
        {
            pServoEn_->set(Level::L_HIGH);
            isPowered_ = false;
        }

        if (updateTimer_.hasPeriodPassed())
        {
            if (isRunning_)
            {
                if (timeoutTimer_.hasOneShotPassed())
                {
                    // Disable servo here
                    pServo_->disable();
                    isRunning_ = false;

                    // Allow for the reading to settle
                    DELAY(100);
                    findState();

#ifdef DEBUG
                    PRINTLN("A: %f", currAngle_);
                    // PRINTLN("State: %d", currState_);
#endif

                    if (currState_ == HopperState::DOOR_AJAR)
                    {
                        hasCommandFailed_ = true;
#ifdef DEBUG
                        // PRINTLN("Movement FAILED!");
#endif
                        // if (!isReturning_)
                        // {
                        //     if (targetState_ == HopperState::DOOR_CLOSED)
                        //     {
                        //         open(Commander::INTERNAL);
                        //     }
                        //     else if (targetState_ == HopperState::DOOR_OPEN)
                        //     {
                        //         close(Commander::INTERNAL);
                        //     }
                        //     isReturning_ = true;
                        // }
                        // else
                        {
                            isReturning_ = false;

                            // Failed to move door
                            handleMovementComplete();
                        }
                    }
                    else if (currState_ == HopperState::DOOR_INVALID)
                    {
#ifdef DEBUG
                        PRINTLN("INVALID STATE!");
                        PRINTLN("Resetting....");  
#endif
                        hasCommandFailed_ = true;
                        handleMovementComplete();
                        DELAY(5000);
                        
                        // reset
                        while(true){}
                    }
                    else
                    {
                        handleMovementComplete();
                        isReturning_ = false;
                    }
                }
                else// if (accelTimer_.hasPeriodPassed())
                {
                    uint16_t servoAngle = pServo_->getAngle();
                    uint16_t nextAngle = servoAngle;
                    uint16_t targetAngle = servoAngle;

                    if (targetState_ == HopperState::DOOR_CLOSED)
                    {
                        targetAngle = pCalibration_->angleClosed;
                    }
                    else if (targetState_ == HopperState::DOOR_OPEN)
                    {
                        targetAngle = pCalibration_->angleOpen;
                    }

                    if (targetAngle < servoAngle)
                    {
                        nextAngle = ((servoAngle - targetAngle) >= angularVelocity_) ?
                                    servoAngle - angularVelocity_ :
                                    targetAngle;

                        pServo_->setAngle(nextAngle);
                    }
                    else if (targetAngle > servoAngle)
                    {
                        nextAngle = ((targetAngle - servoAngle) >= angularVelocity_) ?
                                    servoAngle + angularVelocity_ :
                                    targetAngle;

                        pServo_->setAngle(nextAngle);
                    }
                }
            }
        }
    }

    void DoorController::handleMovementComplete()
    {
        bool success = !hasCommandFailed_ && (currState_ == targetState_);
        if (currCommander_ == Commander::REMOTE)
        {
            const char* resStr = success ? PASS_STR : FAIL_STR;
            char* statusStr = "3\0";
            statusStr[0] = getState() + 0x30;
            pWifiCli->sendWifiCommand(resStr, statusStr);
        }
    }

    int16_t DoorController::readPos()
    {
        uint32_t sum = 0;
        uint8_t readingCount = 0;
        bool success;
        uint16_t reading;
        for (uint8_t i=0; i<NUM_POS_SAMPLES; i++)
        {
            reading = pFeedback_->read(&success);
            if (success)
            {
                sum += reading;
                readingCount++;
            }
        }

        if (readingCount == 0)
        {
            return -1;
        }

        return sum/readingCount;
    }

    void DoorController::test()
    {
    }
}