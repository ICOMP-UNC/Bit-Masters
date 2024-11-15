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

#include "configuration.h"

void EINT0_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT0); /* Clear the external interrupt flag */

    if(LPC_SC->EXTPOLAR & (1 << 0))
    {
        // A rising edge has occured
        // Fire has been detected
        // Config falling edge
        LPC_GPIO0->FIOSET |= MOTOR_POS_PIN; /* Turn on the motor to close the door */
        LPC_GPIO0->FIOSET |= ALARM_PIN; /* Trigger the alarm */
        EXTI_SetPolarity(EXTI_EINT0, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    }
    else
    {
        // A falling edge has occured
        // Fire has been extinguished
        // Config rising edge
        LPC_GPIO0->FIOCLR |= ALARM_PIN; // Deactivate the alarm
        LPC_GPIO0->FIOCLR |= MOTOR_POS_PIN;
        EXTI_SetPolarity(EXTI_EINT0, EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE);
    }
}

void EINT1_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT0); /* Clear the external interrupt flag */
     if(LPC_SC->EXTPOLAR & (1 << 1))
    {
        // A rising edge has occured
        // An intruder has been detected
        // Config falling edge
        LPC_GPIO0->FIOSET |= MOTOR_NEG_PIN; /* Turn on the motor to close the door */
        EXTI_SetPolarity(EXTI_EINT1, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    }
    else
    {
        // A falling edge has occured
        // The intruder is gone
        // Config rising edge
        LPC_GPIO0->FIOCLR |= MOTOR_NEG_PIN;
        EXTI_SetPolarity(EXTI_EINT1, EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE);
    }
}

void TIMER0_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT) == SET)
    {
    	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

        if(LPC_GPIO0->FIOPIN & (MOTOR_POS_PIN || MOTOR_NEG_PIN))
        {
            LPC_GPIO0->FIOCLR |= MOTOR_POS_PIN; /* Turn off the motor */
            LPC_GPIO0->FIOCLR |= MOTOR_NEG_PIN; /* Turn on the motor */
        }
    }
}

void EINT2_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT2); /* Clear the external interrupt flag */

    if(LPC_SC->EXTPOLAR & (1 << 2))
    {
    	NVIC_EnableIRQ(EINT3_IRQn);
    	NVIC_DisableIRQ(EINT0_IRQn);
        EXTI_SetPolarity(EXTI_EINT2, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    }
    else
    {
    	NVIC_DisableIRQ(EINT3_IRQn);
    	NVIC_EnableIRQ(EINT0_IRQn);
        EXTI_SetPolarity(EXTI_EINT2, EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE);
    }

}

void EINT3_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT3); /* Clear the external interrupt flag */
    if(LPC_SC->EXTPOLAR & (1 << 3))
    {
        LPC_GPIO0->FIOSET |= MOTOR_POS_PIN; /* Turn on the motor */
        LPC_GPIO0->FIOCLR |= MOTOR_NEG_PIN;
        // Configurar flanco descendente
        EXTI_SetPolarity(EXTI_EINT3, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    }
    else
    {
        LPC_GPIO0->FIOSET |= MOTOR_NEG_PIN; /* Turn on the motor */
        LPC_GPIO0->FIOCLR |= MOTOR_POS_PIN;
        // Configurar flanco ascendente
        EXTI_SetPolarity(EXTI_EINT3, EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE);
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

    config_ports(); /* Configure the ports */

    config_eint(); /* Configure the external interrupt */

    config_timer_and_match(); /* Configure the timer and match */

    // battery_control(); /* Call the battery control function */

    // SYSTICK_IntCmd(ENABLE); /* Enable the systick timer */

    // SYSTICK_Cmd(ENABLE); /* Enable the systick timer */

    NVIC_SetPriority(EINT2_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 0 */

    NVIC_SetPriority(EINT0_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 1 */

    NVIC_SetPriority(EINT1_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 2 */

    NVIC_SetPriority(EINT3_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 0 */

    NVIC_SetPriority(TIMER0_IRQn, HIGH_PRIORITY); /* Set the priority of the timer 0 */

    NVIC_EnableIRQ(EINT0_IRQn); /* Enable the external interrupt 0 */

    NVIC_EnableIRQ(EINT1_IRQn); /* Enable the external interrupt 1 */

    NVIC_EnableIRQ(EINT2_IRQn); /* Enable the external interrupt 2 */

     /* Enable the external interrupt 2 */

    // NVIC_EnableIRQ(TIMER0_IRQn); /* Enable the timer 0 */

    while (TRUE) /* Infinite loop */
    {
        // Do nothing
    }
    return 0;
}
