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

#include "lpc17xx_systick.h" /* Include the SYSTICK library */
#include "lpc17xx_timer.h"   /* Include the TIMER library */

#include "configuration.h"

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

    if(LPC_SC->EXTPOLAR & (1 << 2))
    {
        // desactivar interrupciones
        // Configurar flanco descendente
    	NVIC_EnableIRQ(EINT3_IRQn);
        EXTI_SetPolarity(EXTI_EINT2, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    }
    else
    {
        // Activar interrupciones
        // Configurar flanco ascendente
    	NVIC_DisableIRQ(EINT3_IRQn);
        EXTI_SetPolarity(EXTI_EINT2, EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE);
    }

    // uint32_t active_int = NVIC->ISER[0]; /* Read the active interrupts */

    // if ((active_int & (1 << EINT0_IRQn)))
    // { /* If the external interrupt 0 is active */
    //     NVIC_DisableIRQ(EINT0_IRQn);
    //     NVIC_DisableIRQ(EINT1_IRQn);
    //     NVIC_DisableIRQ(TIMER0_IRQn);
    //     SYSTICK_IntCmd(DISABLE);
    //     /* Disable the controls*/
    // }
    // else
    // {
    //     NVIC_EnableIRQ(EINT0_IRQn);
    //     NVIC_EnableIRQ(EINT1_IRQn);
    //     NVIC_EnableIRQ(TIMER0_IRQn);
    //     SYSTICK_IntCmd(ENABLE);
    //     /* Enable the controls */
    // }

}

void EINT3_IRQHandler(void)
{
    EXTI_ClearEXTIFlag(EXTI_EINT3); /* Clear the external interrupt flag */
    if(LPC_SC->EXTPOLAR & (1 << 3))
    {
        LPC_GPIO0->FIOSET = MOTOR_POS_PIN; /* Turn on the motor */
        // Configurar flanco descendente
        EXTI_SetPolarity(EXTI_EINT3, EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE);
    }
    else
    {
        LPC_GPIO0->FIOCLR = MOTOR_POS_PIN; /* Turn on the motor */
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

    // config_timer_and_match(); /* Configure the timer and match */

    // battery_control(); /* Call the battery control function */

    // SYSTICK_IntCmd(ENABLE); /* Enable the systick timer */

    // SYSTICK_Cmd(ENABLE); /* Enable the systick timer */

    // NVIC_EnableIRQ(EINT0_IRQn); /* Enable the external interrupt 0 */

    // NVIC_EnableIRQ(EINT1_IRQn); /* Enable the external interrupt 1 */

    NVIC_EnableIRQ(EINT2_IRQn); /* Enable the external interrupt 2 */

     /* Enable the external interrupt 2 */

    // NVIC_EnableIRQ(TIMER0_IRQn); /* Enable the timer 0 */

    NVIC_SetPriority(EINT2_IRQn, HIGH_PRIORITY); /* Set the priority of the external interrupt 0 */

    // NVIC_SetPriority(EINT0_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 1 */

    // NVIC_SetPriority(EINT1_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 2 */

    NVIC_SetPriority(EINT3_IRQn, MID_PRIORITY); /* Set the priority of the external interrupt 0 */

    // NVIC_SetPriority(TIMER0_IRQn, LOW_PRIORITY); /* Set the priority of the timer 0 */

    while (TRUE) /* Infinite loop */
    {
        // Do nothing
    }
    return 0;
}
