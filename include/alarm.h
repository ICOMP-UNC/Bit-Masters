#include "configuration.h" /**< Include the configuration header file */

/**
 * @brief Activates the alarm
 * 
 * This function checks if the alarm is already active and sets the alarm pin high if it is not.
 * 
 */
void activate_alarm(void);

/**
 * @brief Deactivates the alarm
 * 
 * This function checks if the alarm is already inactive and sets the alarm pin low if it is not.
 * 
 */
void deactivate_alarm(void);
