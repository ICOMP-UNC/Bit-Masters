#include "configuration.h" /**< Include the configuration header file */

uint32_t manual_controls = 0; /**< Variable to store the manual control state */

void exti9_5_isr(void){

    /* Check if the interrupt came from the override switch */
    if (exti_get_flag_status(EXTI8)) { 
        if(gpio_get(OVERRIDE_SWITCH_PORT, OVERRIDE_SWITCH_PIN)) {
            manual_controls = 1; /* Set manual_controls to 1 */
        } else {
            manual_controls = 0; /* Set manual_controls to 0 */
        }
    /* Clear the interrupt flag */
    exti_reset_request(EXTI8);
    }
   

    if (exti_get_flag_status(EXTI9)) { /* Check if the interrupt came from the manual switch */
        if(manual_controls) { /* Check if manual_controls is set */
            if(gpio_get(MANUAL_SWITCH_PORT, MANUAL_SWITCH_PIN)) {
                xTaskNotifyGive(open_door_taskHandle); /* Notify the open_door_task */
            } else {
                xTaskNotifyGive(close_door_taskHandle); /* Notify the close_door_task */
            }
        }
        /* Clear the interrupt flag */
        exti_reset_request(EXTI9);
    }
}

void exti15_10_isr(void) {
    /* Check if the interrupt came from the motion sensor */
    if (exti_get_flag_status(EXTI10)) {
        if(gpio_get(MOTION_SENSOR_PORT, MOTION_SENSOR_PIN)) {
            xTaskNotifyGive(close_door_taskHandle); /* Notify the close_door_task */
        }
        else {
            xTaskNotifyGive(open_door_taskHandle); /* Notify the open_door_task */
        }
        /* Clear the interrupt flag */
        exti_reset_request(EXTI10);
    }


    /* Check if the interrupt came from the infrared sensor */
    if (exti_get_flag_status(EXTI11)) {
        if(gpio_get(INFRARED_SENSOR_PORT, INFRARED_SENSOR_PIN)) {
            activate_alarm(); /* Call the activate_alarm function */
        }
        else {
            desactivate_alarm(); /* Call the desactivate_alarm function */
        }
    }

}

