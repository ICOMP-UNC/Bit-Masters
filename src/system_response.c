/**
 * @file system_response.c
 * @brief Functions to control the door motor (open, close, stop).
 * 
 * This file contains functions that manage the operation of the door motor.
 * It provides the functionality to open, close, and stop the motor by 
 * manipulating the appropriate GPIO pins. The motor's direction is controlled 
 * through two pins, one for opening and one for closing the door.
 */

#include "system_response.h"

void close_door(void) {
    gpio_set(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /**< Activate motor for closing. */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /**< Deactivate motor in opening direction. */
}

void open_door(void) {
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /**< Deactivate motor in closing direction. */
    gpio_set(MOTOR_POS_PORT, MOTOR_POS_PIN); /**< Activate motor for opening. */
}

void stop_motor(void) {
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /**< Disable motor in closing direction. */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /**< Disable motor in opening direction. */
}
