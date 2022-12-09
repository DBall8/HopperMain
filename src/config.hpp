#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <stdint.h>


// Device configs
const static uint32_t HOPPER_ID = 1;

const static uint8_t DOOR_ANG_VEL = 1;
const static uint8_t DOOR_TIMEOUT_S = 5;
const static float DOOR_ANGLE_LEEWAY = 10.0f;


// Hardware configs
const static uint32_t TICS_PER_SECOND = 61u;

const static uint16_t MIN_SERVO_MICRO_S = 750;
const static uint16_t MAX_SERVO_MICRO_S = 2300;

const static uint16_t SERVO_FEEDBACK_UPDATE_TICS = 1;

const static uint16_t NUM_POS_SAMPLES = 10;

#endif