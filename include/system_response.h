#include "configuration.h" /**< Include the configuration header file */

/**
 * @brief Open the door by setting the positive motor pin and clearing the negative motor pin
 * 
 * If the motor is already in the open position, the function returns without performing any action.
 */
void open_door(void);

/**
 * @brief Close the door by clearing the positive motor pin and setting the negative motor pin
 * 
 * If the motor is already in the closed position, the function returns without performing any action.
 */
void close_door(void);

/**
 * @brief Stop the motor by clearing both the positive and negative motor pins
 */
void stop_motor(void);

