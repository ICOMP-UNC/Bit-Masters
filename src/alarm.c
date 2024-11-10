#include "alarm.h" /**< Include the alarm header file */

void activate_alarm(void) {
    if(gpio_get(ALARM_PORT, ALARM_PIN)) { /**< Check if the alarm is already active */
        return; /**< Return if the alarm is already active */
    }
    gpio_set(ALARM_PORT, ALARM_PIN); /**< Set the alarm pin high */
}

void deactivate_alarm(void) {
    if(!gpio_get(ALARM_PORT, ALARM_PIN)) { /**< Check if the alarm is already inactive */
        return; /**< Return if the alarm is already inactive */
    }
    gpio_clear(ALARM_PORT, ALARM_PIN); /**< Set the alarm pin low */
}