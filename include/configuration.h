#pragma once

#include "lpc17xx_exti.h"    /* Include the EXTI library */
#include "lpc17xx_gpio.h"    /* Include the GPIO library */
#include "lpc17xx_pinsel.h"  /* Include the PINSEL library */

#define MOTOR_NEG_PIN           ((uint32_t)(1 << 22)) /**< Assign port 0, pin 22 to the LED pin */
#define MOTOR_POS_PIN         ((uint32_t)(1 << 3))  /**< Assign port 0, pin 3 to the motor pin */
#define MANUAL_SWITCH_PIN ((uint32_t)(1 << 13)) /**< Assign port 2, pin 12 to the manual switch pin */
#define OVERRIDE_SWITCH_PIN   ((uint32_t)(1 << 12)) /**< Assign port 2, pin 13 to the door switch pin */

#define INPUT  0 /**< Macro to define the input */
#define OUTPUT 1 /**< Macro to define the output */

#define TRUE  1 /**< Macro to define the true */
#define FALSE 0 /**< Macro to define the false */

#define HIGH_PRIORITY 0 /**< Define the high priority */
#define MID_PRIORITY  1 /**< Define the low priority */
#define LOW_PRIORITY  2 /**< Define the low priority */

/**
 * @brief Configures the ports for the temperature sensor, switch, LED and motor
 *
 * @return void
 */
void config_ports(void);

/**
 * @brief Configures the external interrupt
 *
 * @return void
 */
void config_eint(void);

