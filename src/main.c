// Free RTOS headers
#include "FreeRTOS.h"
#include "task.h"

// Custom headers
#include "configuration.h"
// #include "system_response.h"

// ---------------- Handlers -------------------------

// ---------------- Tasks definitions ----------------

int main(void)
{
    // Hardware configuration
    system_clock_setup();    
    configure_gpio();        
    config_i2c();           
    adc_setup();             
    config_pwm();            
    exti_setup();

    // Task creation

    vTaskStartScheduler();

    // Should never reach this point
    while(TRUE) 
    {

    }

    return SUCCESS;
}