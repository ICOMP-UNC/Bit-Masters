// Custom headers
#include "configuration.h"
// #include "system_response.h"
// #include "alarm_and_notifications.h"

SemaphoreHandle_t adc_semaphore;
TaskHandle_t closeDoorTaskHandle = NULL;
TaskHandle_t openDoorTaskHandle = NULL;

// ---------------- Handlers -------------------------

// ---------------- Tasks definitions ----------------

/**
 * @brief Temperature control task for managing the fan's duty cycle based on temperature readings and
 * to close the door if the temperature is too low.
 *
 * This task is responsible for controlling the fan speed based on the temperature readings. It uses an ADC 
 * to read the temperature and adjusts the duty cycle of the fan accordingly. 
 * The temperature thresholds determine the fan's operation:
 * - Below a certain healthy temperature, the fan is turned off.
 * - Between thresholds, the fan speed is adjusted from low to high.
 * 
 * The task also sends a notification to the `close_door_task` when the temperature goes below a healthy value.
 *
 * @param pvParameters Not used in this task. It is included for compatibility with FreeRTOS task creation.
 *
 * @note This task uses a semaphore (`adc_semaphore`) to synchronize access to the ADC between tasks.
 * @note The duty cycle is updated based on the following thresholds:
 * - `TEMPERATURE_THRESHOLD_LOW`: Low fan speed.
 * - `TEMPERATURE_THRESHOLD_MEDIUM`: Medium fan speed.
 * - `TEMPERATURE_THRESHOLD_HIGH`: High fan speed.
 * - `HEALTHY_TEMPERATURE`: Below this temperature, the fan is off.
 */
void temperature_control_task(void *pvParameters)
{
    uint8_t percentage_duty_cycle = DUTY_CYCLE_OFF; /*If the temperature is lower than 27 degrees Celsius 
    the duty cycle will stay at 1 and the fan will be off */ 
    uint16_t adc_reading = 0;
    while(TRUE)
    {
        if (xSemaphoreTake(adc_semaphore, portMAX_DELAY) == pdTRUE)
        {
            // Make sure one task is using the adc at a time
            adc_reading = read_temperature();
            xSemaphoreGive(adc_semaphore); // Release the sempaphore so that other task can use the ADC
        }
        
        uint16_t temperature = process_temperature(adc_reading);

        if(temperature > TEMPERATURE_THRESHOLD_LOW && temperature < TEMPERATURE_THRESHOLD_MEDIUM)
        {
            percentage_duty_cycle = DUTY_CYCLE_LOW; //Turn on the fan
        }
        else if(temperature >= TEMPERATURE_THRESHOLD_MEDIUM && temperature < TEMPERATURE_THRESHOLD_HIGH)
        {
            percentage_duty_cycle = DUTY_CYCLE_MEDIUM; // Increase fan speed
        } 
        else if(temperature >= TEMPERATURE_THRESHOLD_HIGH)
        {
            percentage_duty_cycle = DUTY_CYCLE_HIGH; // Fan at maximum speed
        }
        else if(temperature > HEALTHY_TEMPERATURE && temperature <= TEMPERATURE_THRESHOLD_LOW)
        {
            percentage_duty_cycle = DUTY_CYCLE_OFF; // Turn off fan
        } 
        else if(temperature < HEALTHY_TEMPERATURE)
        {
            percentage_duty_cycle = DUTY_CYCLE_OFF; // Turn off fan
            // Notify close_door_task
            xTaskNotifyGive(closeDoorTaskHandle);
        }

        set_pwm_duty_cycle(percentage_duty_cycle, PWM_CHANNEL_2);

        vTaskDelay(pdMS_TO_TICKS(ONE_MINUTE_DELAY)); // Wait one minute between temperature readings
    }
}

/**
 * @brief Task to indicate battery level based on PWM duty cycle.
 * 
 * This task periodically checks the battery level and adjusts the PWM duty cycle to indicate 
 * the battery status visually. The task uses predefined thresholds to set the duty cycle 
 * as low, medium, or high, or keeps it off if the battery level is critically low.
 * 
 * - If the battery level is below BATTERY_LEVEL_THRESHOLD_MEDIUM, the duty cycle is set to low.
 * - If the battery level is between BATTERY_LEVEL_THRESHOLD_MEDIUM and BATTERY_LEVEL_THRESHOLD_HIGH, 
 *   the duty cycle is set to medium.
 * - If the battery level is above or equal to BATTERY_LEVEL_THRESHOLD_HIGH, the duty cycle is set to high.
 * 
 * The task then waits for one minute before checking the battery level again.
 * 
 * @param pvParameters Not used in this task, included for compatibility with FreeRTOS task signature.
 * 
 * @note This task should be managed by the FreeRTOS scheduler.
 */
void battery_level_indicator_task(void *pvParameters)
{
    uint8_t percentage_duty_cycle = DUTY_CYCLE_OFF;
    uint8_t battery_level = 0;
    while(TRUE)
    {
        if (xSemaphoreTake(adc_semaphore, portMAX_DELAY) == pdTRUE)
        {
            // Make sure one task is using the adc at a time
            battery_level = get_battery_value();
            xSemaphoreGive(adc_semaphore); // Release the sempaphore so that other task can use the ADC
        }

        if(battery_level < BATTERY_LEVEL_THRESHOLD_MEDIUM)
        {
            percentage_duty_cycle = DUTY_CYCLE_LOW;
        }
        else if(battery_level >= BATTERY_LEVEL_THRESHOLD_MEDIUM && battery_level < BATTERY_LEVEL_THRESHOLD_HIGH)
        {
            percentage_duty_cycle = DUTY_CYCLE_MEDIUM;
        }
        else if(battery_level >= BATTERY_LEVEL_THRESHOLD_HIGH)
        {
            percentage_duty_cycle = DUTY_CYCLE_HIGH;
        }
        set_pwm_duty_cycle(percentage_duty_cycle, PWM_CHANNEL_3); // TODO: Define PWM_CHANNEL_3 in system_response.h

        vTaskDelay(pdMS_TO_TICKS(ONE_MINUTE_DELAY)); // Wait one minute between battery level readings
    }
}

/**
 * @brief Task responsible for closing the door when notified by other tasks.
 * 
 * This task waits for a notification, and once received, it triggers the door closing sequence.
 * It calls the `close_door` function to close the door, waits for the specified time to allow 
 * the door to close properly, and then stops the motor using the `stop_motor` function.
 * The task runs indefinitely and continuously waits for notifications.
 *
 * @param pvParameters A pointer to any parameters passed when creating the task. 
 * This parameter is not used in this task.
 * 
 * @note The task is blocked using `ulTaskNotifyTake` until a notification is received. 
 * After receiving the notification, it performs the door closing operation.
 * @note The task is designed to be notified by other tasks (e.g., the temperature control task) 
 * when a certain condition (e.g., low temperature) occurs.
 */
void close_door_task(void *pvParameters)
{
    while (TRUE)
    {
        // Wait for any notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        close_door();
        vTaskDelay(pdMS_TO_TICKS(DOOR_CLOSING_AND_OPENING_TIME)); // Wait for the door to close
        stop_motor();
    }
}

/**
 * @brief Task responsible for opening the door when notified by other tasks.
 * 
 * This task waits for a notification, and once received, it triggers the door opening sequence.
 * It calls the `open_door` function to open the door, waits for the specified time to allow 
 * the door to open properly, and then stops the motor using the `stop_motor` function.
 * The task runs indefinitely and continuously waits for notifications.
 *
 * @param pvParameters A pointer to any parameters passed when creating the task. 
 * This parameter is not used in this task.
 * 
 * @note The task is blocked using `ulTaskNotifyTake` until a notification is received. 
 * After receiving the notification, it performs the door opening operation.
 * @note The task is designed to be notified by other tasks when a certain condition occurs.
 */
void open_door_task(void *pvParameters)
{
    while (TRUE)
    {
        // Wait for any notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        open_door();
        vTaskDelay(pdMS_TO_TICKS(DOOR_CLOSING_AND_OPENING_TIME)); // Wait for the door to open
        stop_motor();
    }
}

int main(void)
{
    // Hardware configuration
    system_clock_setup();    
    configure_gpio();        
    config_i2c();           
    adc_setup();             
    config_pwm();            
    exti_setup();

    adc_semaphore = xSemaphoreCreateMutex();

    // Task creation
    xTaskCreate(temperature_control_task, TEMPERATURE_CONTROL_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, TEMPERATURE_CONTROL_PRIORITY, NULL);
    xTaskCreate(battery_level_indicator_task, BATTERY_LEVEL_INDICATOR_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, BATTERY_LEVEL_INDICATOR_PRIORITY, NULL);
    xTaskCreate(close_door_task, CLOSE_DOOR_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, CLOSE_DOOR_PRIORITY, &closeDoorTaskHandle);
    xTaskCreate(open_door_task, OPEN_DOOR_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, OPEN_DOOR_PRIORITY, &openDoorTaskHandle);

    vTaskStartScheduler();

    // Should never reach this point
    while(TRUE) 
    {

    }

    return SUCCESS;
}
