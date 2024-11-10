#include "system_response.h" /**< Include the system response header file */

void open_door(void)
{
    /* Open the door */
    if(gpio_get(MOTOR_POS_PORT, MOTOR_POS_PIN) == 1 && gpio_get(MOTOR_NEG_PORT, MOTOR_NEG_PIN) == 0) { /* If the motor is already in the open position, return */
        return;
    }
    gpio_set(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Set the positive motor pin */
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Clear the negative motor pin */
}

void close_door(void)
{
    /* Close the door */
    if(gpio_get(MOTOR_POS_PORT, MOTOR_POS_PIN) == 0 && gpio_get(MOTOR_NEG_PORT, MOTOR_NEG_PIN) == 1) { /* If the motor is already in the closed position, return */
        return;
    }
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Clear the positive motor pin */
    gpio_set(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Set the negative motor pin */
}

void stop_motor(void)
{
    /* Stop the motor */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Clear the positive motor pin */
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Clear the negative motor pin */
}

