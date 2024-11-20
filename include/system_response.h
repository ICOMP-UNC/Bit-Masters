#pragma once

#include "configuration.h" /*<< Includes system configuration and hardware definitions. */

/**
 * @brief Closes the door by activating the motor in the closing direction.
 *
 * This function sets the motor control pins to rotate the motor for closing the door.
 */
void close_door(void);

/**
 * @brief Opens the door by activating the motor in the opening direction.
 *
 * This function sets the motor control pins to rotate the motor for opening the door.
 */
void open_door(void);

/**
 * @brief Stops the motor by disabling both motor control pins.
 *
 * This function clears the motor control pins to stop any ongoing motor operation.
 */
void stop_motor(void);
