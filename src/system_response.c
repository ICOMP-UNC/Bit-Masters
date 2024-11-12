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

uint16_t process_temperature(uint16_t value) 
{
    uint16_t temperature = (value / ADC_RESOLUTION) * TEMP_SCALE_FACTOR;

    return temperature;
}

uint16_t read_temperature(void) {
    // Configure temperature channel
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL_TEMP_SENSOR); // Sequence with one channel

    // Start the conversion
    adc_start_conversion_regular(ADC1);

    // Wait for the conversion to finish
    while (!(ADC1_SR & ADC_SR_EOC));

    // Read and return the result of the conversion
    return adc_read_regular(ADC1);
}

void set_pwm_duty_cycle(uint8_t duty_cycle_param, uint8_t output_channel) {
    // Calculate the PWM compare value based on the duty cycle
    uint32_t pwm_value = (TIMER_PERIOD * duty_cycle_param) / PERCENTAGE_MAX;

    if(output_channel == PWM_CHANNEL_2)
    {
        // Set the duty cycle value for the output channel 2 (OC2) of TIM1
        timer_set_oc_value(TIM1, TIM_OC2, pwm_value);
    }
    else
    {
        // Set the duty cycle value for the output channel 3 (OC3) of TIM1
        timer_set_oc_value(TIM1, TIM_OC3, pwm_value);
    }
}
