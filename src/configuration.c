#include "configuration.h"

void config_ports(void)
{

    PINSEL_CFG_Type pin_cfg; /**< Pin configuration structure */

    pin_cfg.Portnum = PINSEL_PORT_0;           /**< Assign port 0 to the pin configuration */
    pin_cfg.Pinnum = PINSEL_PIN_22;            /**< Assign pin 22 to the pin configuration */
    pin_cfg.Funcnum = PINSEL_FUNC_0;           /**< Assign function 0 to the pin configuration */
    pin_cfg.Pinmode = PINSEL_PINMODE_PULLUP;   /**< Assign pull-up mode to the pin configuration */
    pin_cfg.OpenDrain = PINSEL_PINMODE_NORMAL; /**< Assign normal mode to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /**< Configure the pin */

    /* Configure the positive motor pin */

    pin_cfg.Pinnum = PINSEL_PIN_3; /** Assign pin 3 to the pin configuration */
    PINSEL_ConfigPin(&pin_cfg);    /** Configure the pin */

    /* Configure the motor negative pin */
    pin_cfg.Pinnum = PINSEL_PIN_6; /** Assign pin 6 to the pin configuration */
    PINSEL_ConfigPin(&pin_cfg);    /** Configure the pin */

    pin_cfg.Pinnum = PINSEL_PIN_2;   /** Assign pin 2 to the pin configuration */
    PINSEL_ConfigPin(&pin_cfg); /** Configure the pin */

    pin_cfg.Portnum = PINSEL_PORT_2; /** Assign port 2 to the pin configuration */
    pin_cfg.Pinnum = PINSEL_PIN_12;  /** Assign pin 12 to the pin configuration */
    pin_cfg.Funcnum = PINSEL_FUNC_1; /** Assign function 1 to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /** Configure the pin */

    /* Configure the switch low pin */

    pin_cfg.Pinnum = PINSEL_PIN_10; /** Assign pin 10 to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /** Configure the pin */

    /* Configure the switch high pin */

    pin_cfg.Pinnum = PINSEL_PIN_13; /** Assign pin 11 to the pin configuration */

    PINSEL_ConfigPin(&pin_cfg); /** Configure the pin */

    GPIO_SetDir(PINSEL_PORT_0, (MOTOR_NEG_PIN | MOTOR_POS_PIN | ALARM_PIN), OUTPUT); /* Set the LED and motor pins as output */
    // GPIO_SetDir(PINSEL_PORT_0, I_TEMPERATURE_PIN, INPUT);          /* Set the temperature sensor pin as input */
    GPIO_SetDir(PINSEL_PORT_2, (MANUAL_SWITCH_PIN | OVERRIDE_SWITCH_PIN | FIRE_SENSOR_PIN),
                INPUT); /* Set the switch pins as input */
}

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

    // eint_init_struct.EXTI_Line =
    //     EXTI_EINT1; /** Assign external interrupt 1 to the external interrupt initialization structure */

    // EXTI_Config(&eint_init_struct); /** Initialize the external interrupt */

    eint_init_struct.EXTI_Line =
        EXTI_EINT2; /** Assign external interrupt 2 to the external interrupt initialization structure */

    EXTI_Config(&eint_init_struct); /** Initialize the external interrupt */

    eint_init_struct.EXTI_Line =
        EXTI_EINT3; /** Assign external interrupt 3 to the external interrupt initialization structure */

    EXTI_Config(&eint_init_struct); /** Initialize the external interrupt */
}


void config_timer_and_match(void){
    TIM_TIMERCFG_Type timer_cfg; /**< Define the timer configuration structure */

    timer_cfg.PrescaleOption = TIM_PRESCALE_USVAL; /** Assign the tick value to the prescale option */
    timer_cfg.PrescaleValue = PRESCALE_VALUE;        /** Assign the prescale value to the prescale option */

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timer_cfg); /** Initialize the timer */


    TIM_MATCHCFG_Type match_cfg; /**< Define the match configuration structure */

    match_cfg.MatchChannel = MATCH_CHANNEL; /** Assign match channel 0 to the match configuration structure */
    match_cfg.IntOnMatch = ENABLE; /** Enable the interrupt on match */
    match_cfg.ResetOnMatch = ENABLE; /** Enable the reset on match */
    match_cfg.StopOnMatch = DISABLE; /** Disable the stop on match */
    match_cfg.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE; /** Assign the external match output type to the match
                                                             configuration structure */
    match_cfg.MatchValue = (uint32_t)MATCH_VALUE; /** Assign the match value to the match configuration structure */

    TIM_ConfigMatch(LPC_TIM0, &match_cfg); /** Configure the match */

    TIM_Cmd(LPC_TIM0, ENABLE); /** Enable the timer */
    NVIC_EnableIRQ(TIMER0_IRQn); /** Enable the timer 0 interrupt */
}

