/**
 * @file configuration.c
 * @brief Configuration source file
 */

#include "configuration.h" /**< Include the configuration header file */

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
