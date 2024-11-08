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


