/**
 * @file configuration.c
 * @brief System initialization and configuration functions.
 *
 * This file contains functions to initialize and configure various system
 * components including system clock, GPIO, ADC, USART, PWM, and external interrupts.
 * These functions set up the peripherals, configure their modes, and enable the
 * necessary clocks and DMA for their operation.
 */

#include "configuration.h"

void system_clock_setup(void)
{
    /* Configure the system clock to 72 MHz using PLL and 8 MHz HSE */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void gpio_setup(void)
{
    /* Enable GPIO clocks */
    rcc_periph_clock_enable(RCC_GPIOC); /**< Enable GPIOC peripheral clock */
    rcc_periph_clock_enable(RCC_GPIOA); /**< Enable GPIOA peripheral clock */
    rcc_periph_clock_enable(RCC_GPIOB); /**< Enable GPIOB peripheral clock */

    /* Configure GPIO pins for motor control */
    gpio_set_mode(MOTOR_NEG_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  MOTOR_NEG_PIN); /**< Set MOTOR_NEG_PIN as output with push-pull configuration */
    gpio_set_mode(MOTOR_POS_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  MOTOR_POS_PIN); /**< Set MOTOR_POS_PIN as output with push-pull configuration */

    /* Configure GPIO pins for LED and Fan */
    gpio_set_mode(LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  LED_PIN); /**< Set LED_PIN as output with alternate function push-pull configuration */
    gpio_set_mode(FAN_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  FAN_PIN); /**< Set FAN_PIN as output with alternate function push-pull configuration */

    /* Configure GPIO pin for Alarm */
    gpio_set_mode(ALARM_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  ALARM_PIN); /**< Set ALARM_PIN as output with push-pull configuration */

    /* Configure GPIO pins for motion and fire sensors */
    gpio_set_mode(MOTION_SENSOR_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN,
                  MOTION_SENSOR_PIN); /**< Set MOTION_SENSOR_PIN as input with pull-up/down configuration */
    gpio_set_mode(FIRE_SENSOR_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN,
                  FIRE_SENSOR_PIN); /**< Set FIRE_SENSOR_PIN as input with pull-up/down configuration */

    /* Configure PA0 as input for switch with pull-up/down configuration */
    gpio_set_mode(SWITCH_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN,
                  SWITCH_PIN); /**< Set SWITCH_PIN as input with pull-up/down configuration */

    /* Configure PA1 as input for override button with pull-up/down configuration */
    gpio_set_mode(OVERRIDE_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN,
                  OVERRIDE_PIN); /**< Set OVERRIDE_PIN as input with pull-up/down configuration */
}

void exti_setup(void)
{
    /* Enable AFIO clock (for EXTI configuration) */
    rcc_periph_clock_enable(RCC_AFIO);

    /* Configure EXTI0 (PA0) for both falling and rising edges */
    exti_select_source(EXTI0, SWITCH_PORT);     /* Set PA0 as the EXTI0 source */
    exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH); /* Trigger interrupt on falling and rising edge */
    exti_enable_request(EXTI0);                 /* Enable EXTI0 interrupt */

    /* Configure EXTI1 (PA1) for both falling and rising edges */
    exti_select_source(EXTI1, OVERRIDE_PORT);   /* Set PA1 as the EXTI1 source */
    exti_set_trigger(EXTI1, EXTI_TRIGGER_BOTH); /* Trigger interrupt on falling and rising edge */
    exti_enable_request(EXTI1);                 /* Enable EXTI1 interrupt */

    /* Configure EXTI2 (PA2) for both falling and rising edges */
    exti_select_source(EXTI2, MOTION_SENSOR_PORT); /* Set PA2 as the EXTI2 source */
    exti_set_trigger(EXTI2, EXTI_TRIGGER_BOTH);    /* Trigger interrupt on falling and rising edge */
    exti_enable_request(EXTI2);                    /* Enable EXTI2 interrupt */

    /* Configure EXTI3 (PA3) for both falling and rising edges */
    exti_select_source(EXTI3, FIRE_SENSOR_PORT); /* Set PA3 as the EXTI3 source */
    exti_set_trigger(EXTI3, EXTI_TRIGGER_BOTH);  /* Trigger interrupt on rising and falling edge */
    exti_enable_request(EXTI3);                  /* Enable EXTI3 interrupt */

    /* Enable EXTI0 interrupt in the NVIC */
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    nvic_enable_irq(NVIC_EXTI1_IRQ);
    nvic_enable_irq(NVIC_EXTI2_IRQ);
    nvic_enable_irq(NVIC_EXTI3_IRQ);
}

void configure_adc(void)
{
    /* Enable clock for ADC1 */
    rcc_periph_clock_enable(ADC_CLOCK_ENABLE);

    /* Turn off the ADC before configuring it */
    ADC1_CR2 &= ADC_DISABLE;

    /* Configure the regular sequence length to 1 (single channel) */
    ADC1_SQR1 &= ~(ADC_SEQ_LENGTH_MASK << ADC_SEQ_LENGTH_POS);     /* Clear sequence length bits */
    ADC1_SQR1 |= (ADC_SEQ_LENGTH_1_CHANNEL << ADC_SEQ_LENGTH_POS); /* Set sequence length to 1 channel */

    /* Configure the sample time for channel 4 */
    ADC1_SMPR2 &= ~(ADC_SAMPLE_TIME_MASK << ADC_CHANNEL_4_SAMPLE_POS); /* Clear sample time bits for channel 4 */
    ADC1_SMPR2 |=
        (ADC_SAMPLE_TIME_1_5_CYCLES << ADC_CHANNEL_4_SAMPLE_POS); /* Set sample time to 1.5 cycles for channel 4 */

    /* Configure the sample time for channel 5 */
    ADC1_SMPR2 &= ~(ADC_SAMPLE_TIME_MASK << ADC_CHANNEL_5_SAMPLE_POS); /* Clear sample time bits for channel 5 */
    ADC1_SMPR2 |=
        (ADC_SAMPLE_TIME_1_5_CYCLES << ADC_CHANNEL_5_SAMPLE_POS); /* Set sample time to 1.5 cycles for channel 5 */

    /* Start the regular conversion */
    ADC1_CR2 |= ADC_START_CONVERSION;

    /* Enable DMA mode for ADC */
    ADC1_CR2 |= ADC_ENABLE_DMA;
}

void configure_usart(void)
{
    rcc_periph_clock_enable(RCC_USART1); /* Enable USART1 clock */
    rcc_periph_clock_enable(RCC_GPIOA);  /* Enable GPIOA clock for USART */

    gpio_set_mode(
        USART_TX_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USART_TX_PIN); /* Configure TX pin */
    gpio_set_mode(
        USART_RX_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USART_RX_PIN); /* Configure RX pin as floating input */

    usart_set_baudrate(USART1, UART_BAUD_RATE);             /* Set USART1 baud rate */
    usart_set_databits(USART1, UART_DATA_BITS);             /* Set USART1 data bits */
    usart_set_stopbits(USART1, USART_STOPBITS_1);           /* Set USART1 stop bits */
    usart_set_mode(USART1, USART_MODE_TX);                  /* Set USART1 mode to transmit only */
    usart_set_parity(USART1, USART_PARITY_NONE);            /* Set USART1 parity to none */
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE); /* Disable USART1 flow control */

    usart_enable_tx_dma(USART1); /* Enable USART1 TX DMA */

    usart_enable(USART1); /* Enable USART1 peripheral */
}

void config_pwm(void)
{
    /* Enable the peripheral clock of GPIOA */
    rcc_periph_clock_enable(RCC_TIM4);

    /* Configure the reset and enable the timer peripheral */
    rcc_periph_reset_pulse(RST_TIM4);

    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);

    /* Configure the temporizer to obtain a frequency of 20 kHz */
    timer_set_prescaler(TIM4, PRESCALER_VALUE);

    /* Configure the temporizer to count up to 999 */
    timer_set_period(TIM4, TIMER_PERIOD);

    /* Enable the output channel 1 as PWM */
    timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM2);

    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM2);

    /* Set the output polarity as high */
    timer_set_oc_polarity_high(TIM4, TIM_OC1);

    /* Enable the output for OC2 */
    timer_enable_oc_output(TIM4, TIM_OC3);

    timer_enable_oc_output(TIM4, TIM_OC4);

    /* Enable the break function */
    timer_enable_break_main_output(TIM4);

    /* Enable the temporizer */
    timer_enable_counter(TIM4);
}
