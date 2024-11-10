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

    nvic_enable_irq(NVIC_EXTI15_10_IRQ);

    /* Configure EXTI0 (PA8) for falling edge initially */
    exti_select_source(EXTI8, OVERRIDE_SWITCH_PIN); /* Set PA8 as the EXTI8 source */
    exti_set_trigger(EXTI8, EXTI_TRIGGER_BOTH);  /* Trigger interrupt on rising edge */
    exti_enable_request(EXTI8);                     /* Enable EXTI8 interrupt */

    exti_select_source(EXTI9, MANUAL_SWITCH_PIN); /* Set PA9 as the EXTI9 source */
    exti_set_trigger(EXTI9, EXTI_TRIGGER_BOTH);  /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI9);                     /* Enable EXTI9 interrupt */

    exti_select_source(EXTI10, MOTION_SENSOR_PIN); /* Set PA10 as the EXTI10 source */
    exti_set_trigger(EXTI10, EXTI_TRIGGER_BOTH);  /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI10);                     /* Enable EXTI10 interrupt */

    exti_select_source(EXTI11, INFRARED_SENSOR_PIN); /* Set PA11 as the EXTI11 source */
    exti_set_trigger(EXTI11, EXTI_TRIGGER_BOTH);  /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI11);                     /* Enable EXTI11 interrupt */

}

void config_i2c(void)
{
    // Enable clock for GPIOB and I2C1
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);

    // GPIO pins configuration for SDA and SCL 
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
              GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    // Disable I2C1 before configurating it
    i2c_peripheral_disable(I2C1);

    // I2C basic configuration
    i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
    i2c_set_standard_mode(I2C1);
    i2c_set_trise(I2C1, I2C1_TRISE_100KHZ); // Rising time in standard mode
    i2c_set_ccr(I2C1, I2C1_CCR_100KHZ);  // Set the CCR to 100 kHz

    // Enable I2C to start communication
    i2c_peripheral_enable(I2C1);
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
  
void adc_setup(void) {
      /* Configure ADC */
    adc_power_off(ADC1);                     // Power off the ADC for configuration
    adc_disable_scan_mode(ADC1);             // Single conversion mode (one channel at a time)
    adc_disable_external_trigger_regular(ADC1);
    adc_set_single_conversion_mode(ADC1);    // Single conversion per channel
    adc_set_sample_time(ADC1, ADC_CHANNEL_TEMP_SENSOR, ADC_SMPR_SMP_55DOT5CYC); /*  // Sampling time

    /* Calibrate ADC */
    adc_power_on(ADC1);                      // Power on the ADC
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
    
}
