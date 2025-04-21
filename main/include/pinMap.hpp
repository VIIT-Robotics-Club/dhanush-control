#ifndef PINMAP_HPP
#define PINMAP_HPP
#include "driver/gpio.h"

// tested gpio pins for ballHandler
#define FLYWL_U_PWM GPIO_NUM_12
#define FLYWL_U_DIR GPIO_NUM_9

#define FLYWL_L_PWM GPIO_NUM_11
#define FLYWL_L_DIR GPIO_NUM_10

#define ARM_LIMIT_U GPIO_NUM_5
#define ARM_LIMIT_L GPIO_NUM_6

#define ARM_MOTOR_PWM GPIO_NUM_2
#define ARM_MOTOR_DIR GPIO_NUM_40

#define FLYW_ANGLE_L GPIO_NUM_18
#define FLYW_ANGLE_R GPIO_NUM_17
#define G_FLYW_ANGLE_DIR GPIO_NUM_0     //trial value - pass nothing
#define FINGER_GPIO  GPIO_NUM_16
#define GRIPPER_GPIO  GPIO_NUM_7
    

// #define ARM_MOTOR_PWM GPIO_NUM_7
// #define ARM_MOTOR_DIR GPIO_NUM_9

#define FLY_UPPER_PHASE_A GPIO_NUM_13
#define FLY_LOWER_PHASE_A GPIO_NUM_14

#define ARM_Phase_A GPIO_NUM_8
#define ARM_Phase_B GPIO_NUM_3

#define GRIPPER_PIR GPIO_NUM_45

#define INDEX_FLYW_L 0
#define INDEX_FLYW_U 1
#define INDEX_ARM 2
#define INDEX_FLYW_ANGLE_L 3
#define INDEX_FLYW_ANGLE_R 4

#endif
