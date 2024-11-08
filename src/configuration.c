/**
 * @file configuration.c
 * @brief Configuration source file
 */

#include "configuration.h" /**< Include the configuration header file */

void system_clock_setup(void)
{
    /* Configure the system clock to 72 MHz using PLL and 8 MHz HSE */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void configure_gpio(void) {
    /* Configure the alarm pin as output */
    gpio_set_mode(ALARM_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, ALARM_PIN);
    
    /* Configure the motor pin as output */
    gpio_set_mode(MOTOR_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, MOTOR_PIN);

    /* Configure the manual switch pin as input */
    gpio_set_mode(MANUAL_SWITCH_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN, MANUAL_SWITCH_PIN);

    /* Configure the override switch pin as input */
    gpio_set_mode(OVERRIDE_SWITCH_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN, OVERRIDE_SWITCH_PIN);

    /* Configure the LED pin as output */
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, LED_PIN);

    /* Configure the fan pin as output */
    gpio_set_mode(FAN_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, FAN_PIN);

    /* Configure the temperature sensor pin as input */
    gpio_set_mode(TEMP_SENSOR_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_ANALOG, TEMP_SENSOR_PIN);

    /* Configure the battery level pin as input */
    gpio_set_mode(BATTERY_LEVEL_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_ANALOG, BATTERY_LEVEL_PIN);

    /* Configure the motion sensor pin as input */
    gpio_set_mode(MOTION_SENSOR_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, MOTION_SENSOR_PIN);

    /* Configure the infrared sensor pin as input */
    gpio_set_mode(INFRARED_SENSOR_PORT, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, INFRARED_SENSOR_PIN);

}

void exti_setup(void)
{
    /* Enable AFIO clock */
    rcc_periph_clock_enable(RCC_AFIO);

    /* Enable EXT8 interrupt in the NVIC */
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);

    /* Configure EXTI0 (PA8) for falling edge initially */
    exti_select_source(EXTI8, OVERRIDE_SWITCH_PIN); /* Set PA8 as the EXTI8 source */
    exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);  /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI8);                     /* Enable EXTI8 interrupt */
}

void config_pwm(void) 
{
    /* Enable the peripheral clock of GPIOA */
    rcc_periph_clock_enable(RCC_TIM1);

    /* Configure the reset and enable the timer peripheral */
    rcc_periph_reset_pulse(RST_TIM1);

    /* Configure the temporizer to obtain a frequency of 20 kHz */
    timer_set_prescaler(TIM1, PRESCALER_VALUE); 

    /* Configure the temporizer to count up to 999 */
    timer_set_period(TIM1, TIMER_PERIOD);

    /* Enable the output channel 2 as PWM */
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);

    /* Set the duty cicle at 0% */
    timer_set_oc_value(TIM1, TIM_OC2, duty_cycle); 

    /* Set the output polarity for OC2 as high */
    timer_set_oc_polarity_high(TIM1, TIM_OC2);

    /* Enable the output for OC2 */
    timer_enable_oc_output(TIM1, TIM_OC2);

    /* Enable the output channel 3 as PWM */
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

    /* Set the duty cicle at 0% */
    timer_set_oc_value(TIM1, TIM_OC3, duty_cycle); 

    /* Set the output polarity for OC3 as high */
    timer_set_oc_polarity_high(TIM1, TIM_OC3);

    /* Enable the output for OC3 */
    timer_enable_oc_output(TIM1, TIM_OC3);

    /* Enable the break function */
    timer_enable_break_main_output(TIM1);

    /* Enable the temporizer */
    timer_enable_counter(TIM1);
}


