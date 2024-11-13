/*
 * @file main.c
 * @brief Main file for the project
 *
 * This project involves implementing an embedded system on an LPC1769 board to
 * control the opening and closing mechanism of a shelter door.
 *
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#ifdef __USE_MCUEXPRESSO
#include <cr_section_macros.h> /* The cr_section_macros is specific to the MCUXpresso delivered toolchain */
#endif

#include "lpc17xx_exti.h"    /* Include the EXTI library */
#include "lpc17xx_gpio.h"    /* Include the GPIO library */
#include "lpc17xx_pinsel.h"  /* Include the PINSEL library */
#include "lpc17xx_systick.h" /* Include the SYSTICK library */
#include "lpc17xx_timer.h"   /* Include the TIMER library */

#define O_LED_PIN           ((uint32_t)(1 << 22)) /**< Assign port 0, pin 22 to the LED pin */
#define O_MOTOR_PIN         ((uint32_t)(1 << 3))  /**< Assign port 0, pin 3 to the motor pin */
#define I_SWITCH_LOW        ((uint32_t)(1 << 10)) /**< Assign port 2, pin 10 to the switch low pin */
#define I_SWITCH_HIGH       ((uint32_t)(1 << 11)) /**< Assign port 2, pin 11 to the switch high pin */
#define I_MANUAL_SWITCH_PIN ((uint32_t)(1 << 12)) /**< Assign port 2, pin 12 to the manual switch pin */
#define I_DOOR_SWITCH_PIN   ((uint32_t)(1 << 13)) /**< Assign port 2, pin 13 to the door switch pin */
#define I_TEMPERATURE_PIN   ((uint32_t)(1 << 2))  /**< Assign port 0, pin 1 to the temperature sensor pin */

#define INPUT  0 /**< Macro to define the input */
#define OUTPUT 1 /**< Macro to define the output */

#define TRUE  1 /**< Macro to define the true */
#define FALSE 0 /**< Macro to define the false */

#define SYSTICK_TIME 100 /**< Define the time for the systick timer */

#define PRESCALER_VALUE 1000 /**< Define the prescaler value for the timer */

#define TIMER0_PERIOD 1000000 /**< Define the period for timer 0 */
#define FREC_DIV      10      /**< Define the frequency divider */

#define FAST_BLINK_CNT 2  /**< Define the count for fast blink */
#define SLOW_BLINK_CNT 10 /**< Define the count for slow blink */

#define MATCH_CHANNEL_0 0 /**< Define the match channel 0 */
#define MATCH_CHANNEL_1 1 /**< Define the match channel 1 */

#define HIGH_PRIORITY 0 /**< Define the high priority */
#define MID_PRIORITY  1 /**< Define the low priority */
#define LOW_PRIORITY  2 /**< Define the low priority */

static uint32_t counter = 0;   /**< Time counter */
static uint32_t max_time = 0;  /**< Max time for the LED to be on */
static int close_door = FALSE; /**< Flag to close the door */

/**
 * @brief Configures the ports for the temperature sensor, switch, LED and motor
 *
 * @return void
 */
void config_ports(void)
{

    PINSEL_CFG_Type pin_cfg; /**< Pin configuration structure */

    pin_cfg.Portnum = PINSEL_PORT_0;           /**< Assign port 0 to the pin configuration */
    pin_cfg.Pinnum = PINSEL_PIN_22;            /**< Assign pin 22 to the pin configuration */
    pin_cfg.Funcnum = PINSEL_FUNC_0;           /**< Assign function 0 to the pin configuration */
    pin_cfg.Pinmode = PINSEL_PINMODE_PULLUP;   /**< Assign pull-up mode to the pin configuration */
    pin_cfg.OpenDrain = PINSEL_PINMODE_NORMAL; /**< Assign normal mode to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /**< Configure the pin */

    /* Configure the motor pin */

    pin_cfg.Pinnum = PINSEL_PIN_3; /** Assign pin 3 to the pin configuration */
    PINSEL_ConfigPin(&pin_cfg);    /** Configure the pin */

    /* Configure the temperature sensor pin */

    pin_cfg.Portnum = PINSEL_PORT_0; /** Assign port 0 to the pin configuration */
    pin_cfg.Pinnum = PINSEL_PIN_2;   /** Assign pin 2 to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /** Configure the pin */

    /* Configure the manual switch pin */

    pin_cfg.Portnum = PINSEL_PORT_2; /** Assign port 2 to the pin configuration */
    pin_cfg.Pinnum = PINSEL_PIN_12;  /** Assign pin 12 to the pin configuration */
    pin_cfg.Funcnum = PINSEL_FUNC_1; /** Assign function 1 to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /** Configure the pin */

    /* Configure the switch low pin */

    pin_cfg.Pinnum = PINSEL_PIN_10; /** Assign pin 10 to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /** Configure the pin */

    /* Configure the switch high pin */

    pin_cfg.Pinnum = PINSEL_PIN_11; /** Assign pin 11 to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /** Configure the pin */

    GPIO_SetDir(PINSEL_PORT_0, (O_LED_PIN | O_MOTOR_PIN), OUTPUT); /* Set the LED and motor pins as output */
    GPIO_SetDir(PINSEL_PORT_0, I_TEMPERATURE_PIN, INPUT);          /* Set the temperature sensor pin as input */
    GPIO_SetDir(PINSEL_PORT_2,
                (I_SWITCH_LOW | I_SWITCH_HIGH | I_MANUAL_SWITCH_PIN | I_DOOR_SWITCH_PIN),
                INPUT); /* Set the switch pins as input */
}

/**
 * @brief Configures the external interrupt
 *
 * @return void
 */
void config_eint(void)
{
    EXTI_InitTypeDef eint_init_struct; /**< Define the external interrupt initialization structure */

    eint_init_struct.EXTI_Line =
        EXTI_EINT0; /** Assign external interrupt 0 to the external interrupt initialization structure */
    eint_init_struct.EXTI_Mode =
        EXTI_MODE_EDGE_SENSITIVE; /** Assign edge sensitivity to the external interrupt initialization structure */
    eint_init_struct.EXTI_polarity =
        EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE; /** Assign high active or rising edge sensitivity to the external
                                                     interrupt initialization structure */

    EXTI_Config(&eint_init_struct); /** Initialize the external interrupt */

    eint_init_struct.EXTI_Line =
        EXTI_EINT1; /** Assign external interrupt 1 to the external interrupt initialization structure */

    EXTI_Config(&eint_init_struct); /** Initialize the external interrupt */

    eint_init_struct.EXTI_Line =
        EXTI_EINT2; /** Assign external interrupt 2 to the external interrupt initialization structure */

    EXTI_Config(&eint_init_struct); /** Initialize the external interrupt */

    eint_init_struct.EXTI_Line =
        EXTI_EINT3; /** Assign external interrupt 3 to the external interrupt initialization structure */

    EXTI_Config(&eint_init_struct); /** Initialize the external interrupt */
}

/**
 * @brief Configures the systick timer
 * @param time The time for the systick timer
 *
 * @return void
 */
void config_systick(void)
{
    SYSTICK_InternalInit(SYSTICK_TIME); /* Initialize the systick timer */
}

/**
 * @brief Systick handler
 *
 * If the LED is on, turn it off
 * If the LED is off, turn it on
 * Note: The LED light up when the pin is low
 *
 * @return void
 */
void SysTick_Handler(void)
{
    SYSTICK_ClearCounterFlag(); /* Clear interrupt flag */

    if (counter < max_time) /* If the counter is less than the max time */
    {
        counter++; /* Increment the counter */
    }
    else
    {
        counter = 0; /* Reset the counter */
        if (LPC_GPIO0->FIOPIN & O_LED_PIN)
        {
            LPC_GPIO0->FIOCLR = O_LED_PIN; /* Turn on LED */
        }
        else
        {
            LPC_GPIO0->FIOSET = O_LED_PIN; /* Turn off LED */
        }
    }
}

/**
 * @brief Battery control function
 *
 * If the switch low and switch high are not pressed, blink the LED fast
 * If the switch low is pressed and switch high is not pressed, blink the LED slow
 * If the switch low and switch high are pressed, turn on the LED
 *
 * @return void
 */
void battery_control(void)
{
    if (!(LPC_GPIO2->FIOPIN & I_SWITCH_LOW) &&
        !(LPC_GPIO2->FIOPIN & I_SWITCH_HIGH)) /* If the switch low and switch high are not pressed */
    {
        max_time = (uint32_t)FAST_BLINK_CNT; /* Set the max time as the fast blink count */
        SYSTICK_IntCmd(ENABLE);              /* Enable the systick interrupt */
    }
    else if ((LPC_GPIO2->FIOPIN & I_SWITCH_LOW) &&
             !(LPC_GPIO2->FIOPIN & I_SWITCH_HIGH)) /* If the switch low is pressed and switch high is not pressed */
    {
        max_time = (uint32_t)SLOW_BLINK_CNT; /* Set the max time as the slow blink count */
        SYSTICK_IntCmd(ENABLE);              /* Enable the systick interrupt */
    }
    else
    {
        SYSTICK_IntCmd(DISABLE);       /* Disable the systick interrupt */
        LPC_GPIO0->FIOCLR = O_LED_PIN; /* Turn on LED */
    }
}

/**
 * @brief External interrupt handler
 *
 * Call the battery control function
 *
 * @return void
 */
void EINT0_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT0); /* Clear the external interrupt flag */
    battery_control();              /* Call the battery control function */
}

/**
 * @brief External interrupt handler
 *
 * Call the battery control function
 *
 * @return void
 */
void EINT1_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT1); /* Clear the external interrupt flag */
    battery_control();              /* Call the battery control function */
}

/**
 * @brief External interrupt handler
 *
 * If the external interrupt 2 is triggered, disable the external interrupt 0 and 1
 * If the external interrupt 0 and 1 are triggered, enable the external interrupt 0 and 1
 *
 * @return void
 */
void EINT2_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT2); /* Clear the external interrupt flag */

    uint32_t active_int = NVIC->ISER[0]; /* Read the active interrupts */

    if ((active_int & (1 << EINT0_IRQn)))
    { /* If the external interrupt 0 is active */
        NVIC_DisableIRQ(EINT0_IRQn);
        NVIC_DisableIRQ(EINT1_IRQn);
        NVIC_DisableIRQ(TIMER0_IRQn);
        SYSTICK_IntCmd(DISABLE);
        /* Disable the controls*/
    }
    else
    {
        NVIC_EnableIRQ(EINT0_IRQn);
        NVIC_EnableIRQ(EINT1_IRQn);
        NVIC_EnableIRQ(TIMER0_IRQn);
        SYSTICK_IntCmd(ENABLE);
        /* Enable the controls */
    }
}

void EINT3_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT3); /* Clear the external interrupt flag */
    if (LPC_GPIO0->FIOPIN & O_MOTOR_PIN)
    {
        LPC_GPIO0->FIOCLR = O_MOTOR_PIN; /* Turn off the motor */
    }
    else
    {
        LPC_GPIO0->FIOSET = O_MOTOR_PIN; /* Turn on the motor */
    }
}

void config_timer_and_match(void)
{
    TIM_TIMERCFG_Type timer_cfg_struct; /**< Timer configuration structure */

    timer_cfg_struct.PrescaleOption =
        TIM_PRESCALE_TICKVAL; /* Assign the prescale option to the timer configuration structure */
    timer_cfg_struct.PrescaleValue =
        PRESCALER_VALUE; /* Assign the prescale value to the timer configuration structure */

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timer_cfg_struct); /* Initialize the timer */

    TIM_MATCHCFG_Type match_cfg_struct; /**< Match configuration structure */

    match_cfg_struct.MatchChannel = MATCH_CHANNEL_0; /* Assign the match channel to the match configuration structure */
    match_cfg_struct.IntOnMatch = ENABLE;   /* Assign the interrupt on match to the match configuration structure */
    match_cfg_struct.StopOnMatch = DISABLE; /* Assign the stop on match to the match configuration structure */
    match_cfg_struct.ResetOnMatch = ENABLE; /* Assign the reset on match to the match configuration structure */
    match_cfg_struct.ExtMatchOutputType =
        TIM_EXTMATCH_TOGGLE; /* Assign the external match output type to the match configuration structure */
    match_cfg_struct.MatchValue =
        (uint32_t)TIMER0_PERIOD; /* Assign the match value to the match configuration structure */

    TIM_ConfigMatch(LPC_TIM0, &match_cfg_struct); /* Configure the match */

    match_cfg_struct.MatchChannel = MATCH_CHANNEL_1; /* Assign the match channel to the match configuration structure */
    match_cfg_struct.MatchValue =
        (uint32_t)(TIMER0_PERIOD / FREC_DIV); /* Assign the match value to the match configuration structure */

    TIM_ConfigMatch(LPC_TIM0, &match_cfg_struct); /* Configure the match */

    TIM_Cmd(LPC_TIM0, ENABLE); /* Enable the timer */
}

void TIMER0_IRQHandler(void)
{

    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR1_INT)) /* If the interrupt status is set */
    {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR1_INT); /* Clear the interrupt pending */

        if ((LPC_GPIO0->FIOPIN & I_TEMPERATURE_PIN) &&
            !(close_door)) /* If the temperature sensor is on and the door is not closed */
        {
            LPC_GPIO0->FIOSET = O_MOTOR_PIN; /* Turn on the motor */
            close_door = FALSE;              /* Set the flag close door as false */
        }
        else
        {
            LPC_GPIO0->FIOCLR = O_MOTOR_PIN; /* Turn off the motor */
        }
    }

    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)) /* If the interrupt status is set */
    {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT); /* Clear the interrupt pending */

        if ((LPC_GPIO2->FIOPIN & O_MOTOR_PIN) && !(close_door)) /* If the motor is on and the door is not closed */
        {
            LPC_GPIO0->FIOCLR = O_MOTOR_PIN; /* Turn off the motor */
            close_door = TRUE;               /* Set the flag close door as true */
        }
    }
}

/**
 * @brief Main function
 *
 * @return int
 */
int main(void)
{
    SystemInit(); /* Initialize the system */

    config_systick();

    config_ports(); /* Configure the ports */

    config_eint(); /* Configure the external interrupt */

    config_timer_and_match(); /* Configure the timer and match */

    battery_control(); /* Call the battery control function */

    SYSTICK_IntCmd(ENABLE); /* Enable the systick timer */

    SYSTICK_Cmd(ENABLE); /* Enable the systick timer */

    NVIC_EnableIRQ(EINT0_IRQn); /* Enable the external interrupt 0 */

    NVIC_EnableIRQ(EINT1_IRQn); /* Enable the external interrupt 1 */

    NVIC_EnableIRQ(EINT2_IRQn); /* Enable the external interrupt 2 */

    NVIC_EnableIRQ(EINT3_IRQn); /* Enable the external interrupt 2 */

    NVIC_EnableIRQ(TIMER0_IRQn); /* Enable the timer 0 */

    NVIC_SetPriority(EINT2_IRQn, HIGH_PRIORITY); /* Set the priority of the external interrupt 0 */

    NVIC_SetPriority(EINT0_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 1 */

    NVIC_SetPriority(EINT1_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 2 */

    NVIC_SetPriority(EINT3_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 0 */

    NVIC_SetPriority(TIMER0_IRQn, LOW_PRIORITY); /* Set the priority of the timer 0 */

    while (TRUE) /* Infinite loop */
    {
        // Do nothing
    }
    return 0;
}