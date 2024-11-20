
/**
 * @file main.c
 * @brief Control system for various components (motor, sensors, door, etc.) using FreeRTOS and DMA.
 *
 * This file implements the control system that uses DMA for USART communication,
 * FreeRTOS tasks for controlling various components like a motor, door, and sensors.
 * The system uses ADC to monitor battery levels and temperature, and EXTI interrupts
 * for detecting button presses and sensor inputs.
 */

// Custom modules
#include "configuration.h"   /*<< Includes system configuration and hardware definitions. */
#include "system_response.h" /*<< Includes functions for controlling door motor operations (open, close, stop). */

/**
 * @brief Maximum ADC value (12-bit resolution)
 */
#define ADC_MAX_VALUE 4095

/**
 * @brief Percentage constant (100)
 */
#define PERCENTAGE 100

/**
 * @brief Transmission buffer size for DMA
 */
#define TX_BUFFER_SIZE 128 /**< Buffer size for USART transmission via DMA */

/**
 * @brief Door state CLOSED
 */
#define CLOSED 1

/**
 * @brief Door state OPEN
 */
#define OPEN 0

/**
 * @brief ADC disable macro for ADC1
 */
#define ADC_DISABLE (~ADC_CR2_ADON) /**< Clear the ADON bit to disable the ADC */

/**
 * @brief Define for the conversion factor to scale ADC value to temperature (example for 30°C range)
 */
#define TEMP_CONVERSION_FACTOR 240 /**< Conversion factor for temperature calculation */

/**
 * @brief Threshold temperature in degrees Celsius to trigger actions.
 */
#define TEMPERATURE_THRESHOLD 24

/**
 * @brief Value representing zero temperature to turn off the fan.
 */
#define TEMPERATURE_ZERO 0

/**
 * @brief Delay for one minute in milliseconds.
 */
#define ONE_MINUTE_DELAY 10000

/**
 * @brief Delay for two minutes in milliseconds.
 */
#define TWO_MINUTES_DELAY (ONE_MINUTE_DELAY * 2)

/**
 * @brief Delay for half a minute in milliseconds.
 */
#define HALF_MINUTE_DELAY (ONE_MINUTE_DELAY / 2)

/**
 * @brief Priority for EXTI0 interrupt.
 *
 * Defines the priority level for the EXTI0 interrupt. Lower numbers indicate higher priority.
 */
#define EXTI0_IRQ_PRIORITY 2 /**< Priority for EXTI0 interrupt */

/**
 * @brief Priority for EXTI1 interrupt.
 *
 * Defines the priority level for the EXTI1 interrupt. Lower numbers indicate higher priority.
 */
#define EXTI1_IRQ_PRIORITY 0 /**< Priority for EXTI1 interrupt */

/**
 * @brief Priority for EXTI2 interrupt.
 *
 * Defines the priority level for the EXTI2 interrupt. Lower numbers indicate higher priority.
 */
#define EXTI2_IRQ_PRIORITY 1 /**< Priority for EXTI2 interrupt */

/**
 * @brief Priority for EXTI3 interrupt.
 *
 * Defines the priority level for the EXTI3 interrupt. Lower numbers indicate higher priority.
 */
#define EXTI3_IRQ_PRIORITY 1 /**< Priority for EXTI3 interrupt */

/* Global variables */

/**
 * @brief Buffer for DMA transmission
 */
static char tx_buffer[TX_BUFFER_SIZE]; /**< Transmission buffer */

/**
 * @brief Duty cycle value for PWM control
 */
static uint32_t duty_cycle = 0; /**< PWM duty cycle */

/**
 * @brief Temperature value (from ADC)
 */
static uint32_t temp_value = 0; /**< Temperature value */

/**
 * @brief ADC value buffer for battery and temperature readings
 */
static uint8_t get_batt = 0; /**< Battery value flag */
static uint8_t get_temp = 0; /**< Temperature value flag */

/**
 * @brief ADC data buffer
 */
static uint16_t adc_data = 0; /**< ADC data buffer */

/**
 * @brief Door state: open or closed
 */
static uint8_t door_state = OPEN; /**< Door state */

/**
 * @brief PWM value for fan control
 */
static uint8_t pwm_fan = 0; /**< PWM value for fan control */

/* FreeRTOS Semaphore Handles */

/**
 * @brief Semaphore handle for ADC synchronization
 */
SemaphoreHandle_t xADC = NULL; /**< Semaphore for ADC synchronization */

/**
 * @brief Semaphore handle for door synchronization
 */
SemaphoreHandle_t xDoor = NULL; /**< Semaphore for door synchronization */

/**
 * @brief Semaphore handle for UART synchronization
 */
SemaphoreHandle_t xUART = NULL; /**< Semaphore for UART synchronization */

/* FreeRTOS Task Handles */

/**
 * @brief Task handle for closing door task
 */
TaskHandle_t closeDoorTaskHandle = NULL; /**< Task handle for close door task */

/**
 * @brief Task handle for opening door task
 */
TaskHandle_t openDoorTaskHandle = NULL; /**< Task handle for open door task */

/**
 * @brief Configures DMA1 for ADC and USART1 transfers.
 *
 * This function sets up two DMA channels:
 * - **DMA Channel 1**: Reads 16-bit data from the ADC1 data register and stores it in a memory buffer.
 * - **DMA Channel 4**: Reads 8-bit data from a memory buffer and sends it to USART1 for transmission.
 *
 * @details
 * DMA Channel 1:
 * - Operates in circular mode with a fixed peripheral address and memory destination.
 * - Data size is 16-bit for both peripheral and memory.
 *
 * DMA Channel 4:
 * - Reads from memory (incrementing address) and writes to USART1 data register.
 * - Data size is 8-bit for both peripheral and memory.
 * - Enables interrupt on transfer completion.
 */
void dma_setup(void)
{
    // Enable the clock for DMA1
    rcc_periph_clock_enable(RCC_DMA1);

    // --- DMA1 Channel 1 Configuration (ADC) ---
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1)); /* ADC1 data register as the source. */
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)&adc_data);         /* Memory buffer as destination. */
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);        /* 16-bit data size for peripheral. */
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);            /* 16-bit data size for memory. */
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1);                           /* Transfer 1 data unit per cycle. */
    dma_disable_memory_increment_mode(DMA1, DMA_CHANNEL1);                   /* Memory address is fixed. */
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);               /* Peripheral address is fixed. */
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);              /* Set high priority for ADC transfers. */
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1); /* Enable circular mode for continuous transfers. */
    dma_enable_channel(DMA1, DMA_CHANNEL1);       /* Activate DMA1 Channel 1. */

    // --- DMA1 Channel 4 Configuration (USART1 TX) ---
    dma_disable_channel(DMA1, DMA_CHANNEL4); /* Ensure Channel 4 is disabled during setup. */
    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR); /* USART1 data register as the destination. */
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)tx_buffer);      /* TX buffer as the data source. */
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);      /* 8-bit data size for peripheral. */
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);          /* 8-bit data size for memory. */
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);                         /* Memory as the source of data. */
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);                 /* Enable memory address increment. */
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_MEDIUM);    /* Set medium priority for USART1 transfers. */
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, TX_BUFFER_SIZE); /* Number of bytes to transfer. */
    dma_enable_channel(DMA1, DMA_CHANNEL4);                     /* Activate DMA1 Channel 4. */

    // Enable transfer complete interrupt for DMA1 Channel 4
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ); /* Enable NVIC interrupt for DMA1 Channel 4. */
}

/**
 * @brief Changes the ADC sequence based on the active channel (battery or temperature)
 */
void change_adc_sequence(void)
{
    ADC1_CR2 &= ADC_DISABLE; /**< Disable ADC before changing the sequence */

    dma_disable_channel(DMA1, DMA_CHANNEL1); /**< Disable DMA channel */

    if (get_batt == TRUE)
    {
        /* Configure channel 4 (battery) as the first channel in the sequence */
        ADC1_SQR3 &= ~ADC_SEQUENCE_CHANNEL_MASK;                     /**< Clear the bits for channel configuration */
        ADC1_SQR3 |= (ADC_CHANNEL_BATT << ADC_SEQUENCE_CHANNEL_POS); /**< Set channel 4 (battery) */
    }

    if (get_temp == TRUE)
    {
        /* Configure channel 5 (temperature) as the first channel in the sequence */
        ADC1_SQR3 &= ~ADC_SEQUENCE_CHANNEL_MASK;                     /**< Clear the bits for channel configuration */
        ADC1_SQR3 |= (ADC_CHANNEL_TEMP << ADC_SEQUENCE_CHANNEL_POS); /**< Set channel 5 (temperature) */
    }

    ADC1_CR2 |= ADC_ENABLE; /**< Enable ADC after changing the sequence */

    dma_enable_channel(DMA1, DMA_CHANNEL1); /**< Enable DMA channel */
}

/**
 * @brief Reads the current battery value from the ADC
 */
uint16_t get_battery_value(void)
{
    if (xSemaphoreTake(xADC, portMAX_DELAY) == pdTRUE)
    {
        get_batt = TRUE;                   /**< Set the flag to read the battery */
        get_temp = FALSE;                  /**< Clear the flag for temperature reading */
        change_adc_sequence();             /**< Change ADC sequence for battery reading */
        adc_start_conversion_direct(ADC1); /**< Start ADC conversion for battery */
        xSemaphoreGive(xADC);              /**< Release the semaphore */
    }
    return ((adc_data * PERCENTAGE) / ADC_MAX_VALUE); /**< Return the battery value as a percentage */
}

/**
 * @brief Reads the current temperature value from the ADC
 */
uint16_t get_temp_value(void)
{
    if (xSemaphoreTake(xADC, portMAX_DELAY) == pdTRUE)
    {
        get_batt = FALSE;                  /**< Clear the flag for battery reading */
        get_temp = TRUE;                   /**< Set the flag to read the temperature */
        change_adc_sequence();             /**< Change ADC sequence for temperature reading */
        adc_start_conversion_direct(ADC1); /**< Start ADC conversion for temperature */
        xSemaphoreGive(xADC);              /**< Release the semaphore */
    }
    return ((adc_data * TEMP_CONVERSION_FACTOR) / ADC_MAX_VALUE); /**< Return the temperature value in Celsius */
}

/**
 * @brief Sends a formatted string via USART using DMA.
 *
 * This function sends a formatted string over USART. If the DMA transfer is currently busy,
 * the function returns without performing the operation. It uses `vsnprintf` to format the
 * string and then starts a DMA transfer to send the data.
 *
 * @param str The format string (similar to printf).
 * @param ... Additional arguments for formatting.
 */
void usart_send_string(const char* str, ...)
{
    if (!(xSemaphoreTake(xUART, portMAX_DELAY) == pdTRUE))
    { /* Check if the UART semaphore is available */
        return;
    }

    char formatted_buffer[TX_BUFFER_SIZE];                                           /* Buffer for formatted string */
    va_list args;                                                                    /* Variable argument list */
    va_start(args, str);                                                             /* Initialize variable arguments */
    uint16_t len = vsnprintf(formatted_buffer, sizeof(formatted_buffer), str, args); /* Format the string */
    va_end(args);                                                                    /* End variable arguments */

    if (len < 0 || len >= TX_BUFFER_SIZE)
    { /* Check for errors in formatting */
        return;
    }

    memcpy(tx_buffer, formatted_buffer, len); /* Copy the formatted string to the TX buffer */

    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)tx_buffer); /* Set the memory address for DMA transfer */
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, len);                 /* Set the number of bytes to transfer */

    dma_enable_channel(DMA1, DMA_CHANNEL4); /* Enable DMA channel 4 */
}

/* ----------------------- Task Definitions ------------------------------- */

/**
 * @brief Task to control the temperature and manage the fan speed via PWM.
 *
 * This task continuously monitors the temperature, adjusts the PWM duty cycle
 * for the fan, and sends notifications to close the door if the temperature is below
 * a specified threshold.
 */
void temperature_control_task(void* pvParameters __attribute__((unused)))
{
    while (TRUE)
    {
        temp_value = get_temp_value();

        if (temp_value < TEMPERATURE_THRESHOLD)
        {
            pwm_fan = TEMPERATURE_ZERO; /* Turn off fan*/
            /* Notify to close the door */
            xTaskNotifyGive(closeDoorTaskHandle);
        }
        else
        {
            pwm_fan = temp_value; /* Set fan speed based on temperature */
        }

        usart_send_string("Temperatura actual: %d°C\r\n", temp_value); /* Send temperature via serial */
        timer_set_oc_value(
            TIM4, TIM_OC4, TIMER_PERIOD - (TIMER_PERIOD * temp_value) / PERCENTAGE); /* Set PWM duty cycle */

        vTaskDelay(TWO_MINUTES_DELAY / portTICK_PERIOD_MS); /* Wait for two minutes between readings */
    }
}

/**
 * @brief Task to monitor and indicate battery level via PWM.
 *
 * This task reads the battery level, calculates the corresponding PWM duty cycle,
 * and sends the battery percentage through the serial port.
 */
void battery_level_indicator_task(void* pvParameters __attribute__((unused)))
{
    while (TRUE)
    {
        duty_cycle = get_battery_value();                                 /* Get battery percentage */
        usart_send_string("Porcentaje de bateria: %d%%\r\n", duty_cycle); /* Send battery percentage via serial */
        uint32_t pwm_value = TIMER_PERIOD - (TIMER_PERIOD * duty_cycle) / PERCENTAGE;
        timer_set_oc_value(TIM4, TIM_OC3, pwm_value); /* Set PWM duty cycle */

        vTaskDelay(ONE_MINUTE_DELAY / portTICK_PERIOD_MS); /* Wait for one minute between readings */
    }
}

/**
 * @brief Task to close the door when notified.
 *
 * This task waits for a notification to close the door, ensuring mutual exclusion
 * with a semaphore. After closing the door, it stops the motor.
 */
void close_door_task(void* pvParameters __attribute__((unused)))
{
    while (TRUE)
    {
        /* Wait for a notification */
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
        {
            if (xSemaphoreTake(xDoor, portMAX_DELAY) == pdTRUE)
            {
                if (door_state == OPEN)
                {
                    door_state = CLOSED;                                /* Set door state to closed */
                    close_door();                                       /* Close the door */
                    vTaskDelay(HALF_MINUTE_DELAY / portTICK_PERIOD_MS); /* Wait for door to close */
                    stop_motor();                                       /* Stop the motor */
                }
                xSemaphoreGive(xDoor); /* Release the semaphore */
            }
        }
    }
}

/**
 * @brief Task to open the door when notified.
 *
 * This task waits for a notification to open the door, ensuring mutual exclusion
 * with a semaphore. After opening the door, it stops the motor.
 */
void open_door_task(void* pvParameters __attribute__((unused)))
{
    while (TRUE)
    {
        /* Wait for a notification */
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
        {
            if (xSemaphoreTake(xDoor, portMAX_DELAY) == pdTRUE)
            {
                if (door_state == CLOSED)
                {
                    door_state = OPEN;                                  /* Set door state to open */
                    open_door();                                        /* Open the door */
                    vTaskDelay(HALF_MINUTE_DELAY / portTICK_PERIOD_MS); /* Wait for door to open */
                    stop_motor();                                       /* Stop the motor */
                }
                xSemaphoreGive(xDoor); /* Release the semaphore */
            }
        }
    }
}

/**
 * @brief Application hook function for stack overflow handling.
 *
 * This function is called when a stack overflow occurs in any task. It
 * enters an infinite loop, halting further execution. It is used to
 * debug stack overflow issues.
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask __attribute__((unused)),
                                   char* pcTaskName __attribute__((unused)))
{
    while (TRUE)
        ; /**< Infinite loop to halt the system on stack overflow */
}

/**
 * @brief Configures binary semaphores for ADC and door control.
 *
 * This function creates two binary semaphores used for synchronization:
 * one for controlling ADC access (`xADC`) and one for controlling door
 * operations (`xDoor`). If the semaphores cannot be created, it enters
 * an infinite loop and sends an error message over USART.
 */
void config_semaphore(void)
{
    // Create a binary semaphore for ADC access
    xADC = xSemaphoreCreateBinary();
    if (xADC == NULL)
    {
        while (TRUE)
        {                               /**< Infinite loop in case of semaphore creation failure */
            usart_send_string("ERROR"); /**< Send error message if semaphore creation fails */
        }
    }
    xSemaphoreGive(xADC); /**< Release the semaphore to allow ADC access */

    // Create a binary semaphore for door control
    xDoor = xSemaphoreCreateBinary();
    if (xDoor == NULL)
    {
        while (TRUE)
        {                               /**< Infinite loop in case of semaphore creation failure */
            usart_send_string("ERROR"); /**< Send error message if semaphore creation fails */
        }
    }
    xSemaphoreGive(xDoor); /**< Release the semaphore to allow door control */

    // Create a binary semaphore for UART access
    xUART = xSemaphoreCreateBinary();
    if (xUART == NULL)
    {
        while (TRUE)
        {                               /**< Infinite loop in case of semaphore creation failure */
            usart_send_string("ERROR"); /**< Send error message if semaphore creation fails */
        }
    }
    xSemaphoreGive(xUART); /**< Release the semaphore to allow UART access */
}

/* --------------------- Interrupt service routines -------------------------- */

/**
 * @brief DMA1 Channel 4 interrupt service routine.
 *
 * Handles the transfer complete interrupt for USART1 TX DMA transfer.
 * - Clears the interrupt flag.
 * - Disables the DMA channel to prevent further transfers.
 * - Gives the xUART semaphore to indicate the transfer is complete.
 */
void dma1_channel4_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF); /* Clear transfer complete flag. */
        dma_disable_channel(DMA1, DMA_CHANNEL4);                 /* Disable DMA1 Channel 4. */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;           /* Task notification not required. */
        xSemaphoreGiveFromISR(xUART, &xHigherPriorityTaskWoken); /* Give the UART semaphore. */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);            /* Switch to higher priority task if required. */
    }
}

/**
 * @brief ISR for EXTI0 interrupt (door control based on edge detection).
 *
 * This ISR toggles between opening and closing the door based on the
 * edge detection state (falling or rising). It notifies the appropriate
 * FreeRTOS task.
 */
void exti0_isr(void)
{
    /* Clear the EXTI0 interrupt flag */
    exti_reset_request(EXTI0);

    /* Check if the interrupt is the switch pin is low */
    if (!(gpio_get(SWITCH_PORT, SWITCH_PIN)))
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* Notify the task to close the door */
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* Notify the task to open the door */
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief ISR for EXTI1 interrupt (manual control toggle).
 *
 * Toggles between enabling or disabling manual controls. This is communicated
 * via USART messages and updates the NVIC interrupt configuration.
 */
void exti1_isr(void)
{
    /* Clear the EXTI1 interrupt flag */
    exti_reset_request(EXTI1);

    /* Check if the interrupt is the override port is inactive */
    if (!(gpio_get(OVERRIDE_PORT, OVERRIDE_PIN)))
    {
        /* Disable EXTI0 interrupt and enable EXTI2, EXTI3 */
        nvic_disable_irq(NVIC_EXTI0_IRQ); // Disable manual controls
        nvic_enable_irq(NVIC_EXTI2_IRQ);  // Enable motion based controls
        nvic_enable_irq(NVIC_EXTI3_IRQ);  // Enable fire based controls

        /* Notify manual controls deactivation */
        usart_send_string("Controles manuales desactivados\r\n");
    }
    else
    {
        /* Enable EXTI0 interrupt and disable EXTI2, EXTI3 */
        nvic_enable_irq(NVIC_EXTI0_IRQ);  // Enable manual controls
        nvic_disable_irq(NVIC_EXTI2_IRQ); // Disable automatic controls
        nvic_disable_irq(NVIC_EXTI3_IRQ);

        /* Notify manual controls activation */
        usart_send_string("Controles manuales activados\r\n");
    }
}

/**
 * @brief ISR for EXTI2 interrupt (safe zone detection).
 *
 * This ISR handles the detection of safe zones, notifying the
 * appropriate FreeRTOS task. The door opens or closes depending on
 * the detection, and a message is sent via USART to indicate the zone state.
 */
void exti2_isr(void)
{
    /* Clear the EXTI2 interrupt flag */
    exti_reset_request(EXTI2);

    /* Check if the interrupt is the motion sensor pin is low */
    if (!gpio_get(MOTION_SENSOR_PORT, MOTION_SENSOR_PIN))
    {
        /* Notify the safe zone detection and open the door */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        /* Notify the danger zone and close the door */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief ISR for EXTI3 interrupt (fire detection and alarm activation).
 *
 * Handles fire detection via an interrupt, notifying the appropriate task
 * to open or close the door. It also triggers the alarm.
 */
void exti3_isr(void)
{
    /* Clear the EXTI3 interrupt flag */
    exti_reset_request(EXTI3);

    /* Check if the interrupt is the fire sensor pin is at a low state */
    if (!gpio_get(FIRE_SENSOR_PORT, FIRE_SENSOR_PIN))
    {
        /* Deactivate the alarm */
        gpio_set(ALARM_PORT, ALARM_PIN); /* Turn off the alarm on rising edge */

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        /* Reactivate the alarm */
        gpio_clear(ALARM_PORT, ALARM_PIN);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Main function that initializes the system and starts the scheduler.
 *
 * This function sets up the system clock, GPIO pins, EXTI for interrupt handling,
 * ADC, USART, PWM, DMA, and semaphores. It then creates tasks for controlling
 * temperature, battery level, and door operations. Finally, it starts the FreeRTOS
 * scheduler, which manages the tasks.
 *
 * @return int 0 if program execution completes successfully (it shouldn't, since
 * the program relies on the FreeRTOS scheduler).
 */
int main(void)
{
    // Initialize system components
    system_clock_setup(); /**< Configure system clock */
    gpio_setup();         /**< Configure GPIO pins for the system */
    exti_setup();         /**< Configure EXTI for button press detection */
    configure_adc();      /**< Configure ADC for reading sensor data */
    configure_usart();    /**< Configure USART for serial communication */
    config_pwm();         /**< Configure PWM for controlling actuators */
    dma_setup();          /**< Configure DMA for data transfer */

    // Initialize semaphores for synchronization
    config_semaphore();

    // Set interrupt priorities for EXTI lines
    nvic_set_priority(NVIC_EXTI0_IRQ, EXTI0_IRQ_PRIORITY); /**< Set EXTI0 interrupt priority */
    nvic_set_priority(NVIC_EXTI1_IRQ, EXTI1_IRQ_PRIORITY); /**< Set EXTI1 interrupt priority */
    nvic_set_priority(NVIC_EXTI2_IRQ, EXTI2_IRQ_PRIORITY); /**< Set EXTI2 interrupt priority */
    nvic_set_priority(NVIC_EXTI3_IRQ, EXTI3_IRQ_PRIORITY); /**< Set EXTI3 interrupt priority */

    // Task creation for different functionalities
    xTaskCreate(temperature_control_task,
                TEMPERATURE_CONTROL_TASK_NAME,
                TEMPERATURE_CONTROL_STACK_SIZE,
                NULL,
                TEMPERATURE_CONTROL_PRIORITY,
                NULL);
    xTaskCreate(battery_level_indicator_task,
                BATTERY_LEVEL_INDICATOR_TASK_NAME,
                BATTERY_LEVEL_INDICATOR_STACK_SIZE,
                NULL,
                BATTERY_CONTROL_PRIORITY,
                NULL);
    xTaskCreate(
        close_door_task, CLOSE_DOOR_TASK_NAME, CLOSE_DOOR_STACK_SIZE, NULL, CLOSE_DOOR_PRIORITY, &closeDoorTaskHandle);
    xTaskCreate(
        open_door_task, OPEN_DOOR_TASK_NAME, OPEN_DOOR_STACK_SIZE, NULL, OPEN_DOOR_PRIORITY, &openDoorTaskHandle);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Main loop (in this case, the program relies on interrupts for operation)
    while (TRUE)
    {
        // The main loop does nothing since tasks and interrupts handle the operation
    }

    return 0; /**< Return statement (will never be reached due to scheduler) */
}
