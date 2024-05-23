#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <stdint.h>


// Device configs
const static uint8_t DOOR_ANG_VEL = 1;
const static uint8_t DOOR_TIMEOUT_S = 5;
const static uint8_t CALIBRATION_TIMEOUT_S = 120;
const static float DOOR_ANGLE_LEEWAY = 20.0f;


// Hardware configs
const static uint32_t TICS_PER_SECOND = 61u;

// Mess with these to try and get a wider range
const static uint16_t MIN_SERVO_MICRO_S = 750;
const static uint16_t MAX_SERVO_MICRO_S = 2300;

const static uint16_t SERVO_FEEDBACK_UPDATE_TICS = 1;

// Sample position
const static uint16_t NUM_POS_SAMPLES = 10;
const static uint16_t SAMPLE_DELAY_MS = 10;

// Mess with MIN/MAX SERVO_MICRO_S instead of these
const static int16_t MIN_SERVO_ANGLE = 0;
const static int16_t MAX_SERVO_ANGLE = 180;

#endif