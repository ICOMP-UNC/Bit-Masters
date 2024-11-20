
/**
 * @file main.c
 * @brief Control system for various components (motor, sensors, door, etc.) using FreeRTOS and DMA.
 *
 * This file implements the control system that uses DMA for USART communication, 
 * FreeRTOS tasks for controlling various components like a motor, door, and sensors.
 * The system uses ADC to monitor battery levels and temperature, and EXTI interrupts 
 * for detecting button presses and sensor inputs.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <stdio.h>
#include <stdarg.h> 

// FreeRTOS headers
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Constants */

/**
 * @brief Define for logical TRUE
 */
#define TRUE    1

/**
 * @brief Define for logical FALSE
 */
#define FALSE   0

/**
 * @brief Define for falling edge detection
 */
#define FALLING 0

/**
 * @brief Define for rising edge detection
 */
#define RISING  1

/* Pin Definitions */

/** 
 * @brief Port and pin configuration for motor negative terminal
 */
#define MOTOR_NEG_PORT    GPIOA /**< Port A connected to motor negative terminal */
#define MOTOR_NEG_PIN     GPIO7  /**< PA7 connected to motor negative terminal */

/** 
 * @brief Port and pin configuration for motor positive terminal
 */
#define MOTOR_POS_PORT    GPIOA /**< Port A connected to motor positive terminal */
#define MOTOR_POS_PIN     GPIO6  /**< PA6 connected to motor positive terminal */

/** 
 * @brief Port and pin configuration for switch (manual control)
 */
#define SWITCH_PORT GPIOA /**< Port A connected to button (switch) */
#define SWITCH_PIN  GPIO0 /**< PA0 connected to button (switch) */

/** 
 * @brief Port and pin configuration for override control
 */
#define OVERRIDE_PORT GPIOA /**< Port A connected to button (override) */
#define OVERRIDE_PIN  GPIO1 /**< PA1 connected to button (override) */

/** 
 * @brief Port and pin configuration for motion sensor
 */
#define MOTION_SENSOR_PORT GPIOA /**< Port A connected to motion sensor */
#define MOTION_SENSOR_PIN  GPIO2 /**< PA2 connected to motion sensor */

/** 
 * @brief Port and pin configuration for fire sensor
 */
#define FIRE_SENSOR_PORT GPIOA /**< Port A connected to fire sensor */
#define FIRE_SENSOR_PIN  GPIO3 /**< PA3 connected to fire sensor */

/** 
 * @brief Port and pin configuration for alarm
 */
#define ALARM_PORT GPIOA /**< Port A connected to alarm */
#define ALARM_PIN  GPIO8 /**< PA8 connected to alarm */

/** 
 * @brief Port and pin configuration for LED
 */ 
#define LED_PORT GPIOB /**< Port B connected to LED */
#define LED_PIN  GPIO8 /**< PB8 connected to LED */

/** 
 * @brief Port and pin configuration for fan
 */
#define FAN_PORT GPIOB /**< Port B connected to fan */
#define FAN_PIN  GPIO9 /**< PB9 connected to fan */

/**
 * @brief Prescaler value for timer
 */
#define PRESCALER_VALUE 71 

/**
 * @brief Timer period for the timer
 */
#define TIMER_PERIOD 999

/**
 * @brief Maximum ADC value (12-bit resolution)
 */
#define ADC_MAX_VALUE 4095

/**
 * @brief Percentage constant (100)
 */
#define PERCENTAGE 100

/** 
 * @brief Buffer size for ADC data storage
 */
#define ADC_BUFFER_SIZE 128

/** 
 * @brief Transmission buffer size for DMA
 */
#define TX_BUFFER_SIZE 128  /**< Buffer size for USART transmission via DMA */

/* Task names */

/** 
 * @brief Task name for temperature control
 */
#define TEMPERATURE_CONTROL_TASK_NAME       "temperature_control"

/** 
 * @brief Task name for battery control
 */
#define BATTERY_LEVEL_INDICATOR_TASK_NAME           "battery_control"

/** 
 * @brief Task name for door control to close
 */
#define CLOSE_DOOR_TASK_NAME                "close_door"

/** 
 * @brief Task name for door control to open
 */
#define OPEN_DOOR_TASK_NAME                 "open_door"

/* Task priorities */

/** 
 * @brief Priority for temperature control task
 */
#define TEMPERATURE_CONTROL_PRIORITY        tskIDLE_PRIORITY + 2

/** 
 * @brief Priority for battery control task
 */
#define BATTERY_CONTROL_PRIORITY            tskIDLE_PRIORITY + 1

/** 
 * @brief Priority for door close task
 */
#define CLOSE_DOOR_PRIORITY                 tskIDLE_PRIORITY + 2

/** 
 * @brief Priority for door open task
 */
#define OPEN_DOOR_PRIORITY                  tskIDLE_PRIORITY + 2

/** 
 * @brief Door states: CLOSED or OPEN
 */
#define CLOSED 1
#define OPEN 0

/** 
 * @brief ADC clock enable macro for ADC1
 */
#define ADC_CLOCK_ENABLE             RCC_ADC1           /**< Enable the clock for ADC1 */

/** 
 * @brief ADC disable macro for ADC1
 */
#define ADC_DISABLE                  (~ADC_CR2_ADON)    /**< Clear the ADON bit to disable the ADC */

/** 
 * @brief Mask for the sequence length bits in ADC1_SQR1 register
 */
#define ADC_SEQ_LENGTH_MASK          0xF                /**< Mask for the sequence length bits */

/** 
 * @brief Position of the sequence length bits in ADC1_SQR1 register
 */
#define ADC_SEQ_LENGTH_POS           20                 /**< Position of the sequence length bits */

/** 
 * @brief Sequence length set to 1 channel in ADC configuration
 */
#define ADC_SEQ_LENGTH_1_CHANNEL     0x0                /**< Set the sequence length to 1 channel */

/** 
 * @brief Mask for the sample time bits in ADC1_SMPR2 register
 */
#define ADC_SAMPLE_TIME_MASK         0x7                /**< Mask for the sample time bits */

/** 
 * @brief Position of the sample time bits for channel 4 in ADC1_SMPR2 register
 */
#define ADC_CHANNEL_4_SAMPLE_POS     12                 /**< Position of the sample time bits for channel 4 */

/** 
 * @brief Position of the sample time bits for channel 5 in ADC1_SMPR2 register
 */
#define ADC_CHANNEL_5_SAMPLE_POS     15                 /**< Position of the sample time bits for channel 5 */

/** 
 * @brief Sample time of 1.5 cycles for ADC channel configuration
 */
#define ADC_SAMPLE_TIME_1_5_CYCLES   ADC_SMPR_SMP_1DOT5CYC /**< Set sample time to 1.5 cycles */

/** 
 * @brief ADC conversion start macro for regular conversion
 */
#define ADC_START_CONVERSION         ADC_CR2_SWSTART    /**< Set the SWSTART bit to begin conversion */

/** 
 * @brief ADC DMA enable macro to enable DMA for ADC
 */
#define ADC_ENABLE_DMA               ADC_CR2_DMA        /**< Enable DMA mode for ADC */

/** 
 * @brief Define for enabling the ADC (ADC1)
 */
#define ADC_ENABLE                  ADC_CR2_ADON      /**< Set the ADON bit to enable ADC1 */

/** 
 * @brief Define for disabling the ADC (ADC1)
 */
#define ADC_DISABLE                 (~ADC_CR2_ADON)   /**< Clear the ADON bit to disable ADC1 */

/** 
 * @brief Define for the ADC sequence channel mask in the ADC1_SQR3 register
 */
#define ADC_SEQUENCE_CHANNEL_MASK   0x1F              /**< Mask for the sequence channel bits */

/** 
 * @brief Define for the position of the sequence channel in the ADC1_SQR3 register
 */
#define ADC_SEQUENCE_CHANNEL_POS    0                 /**< Position of the channel in the sequence */

/** 
 * @brief Define for the ADC channel used to read the battery
 */
#define ADC_CHANNEL_BATT            4                 /**< ADC channel for reading battery voltage */

/** 
 * @brief Define for the ADC channel used to read the temperature
 */
#define ADC_CHANNEL_TEMP            5                 /**< ADC channel for reading temperature */

/** 
 * @brief Define for the conversion factor to scale ADC value to temperature (example for 30°C range)
 */
#define TEMP_CONVERSION_FACTOR      240                /**< Conversion factor for temperature calculation */

/**
 * @brief Baud rate for UART communication.
 */
#define UART_BAUD_RATE 9600 /**< UART baud rate */

/**
 * @brief Number of data bits in UART communication.
 */
#define UART_DATA_BITS 8 /**< UART data bits */

/**
 * @brief GPIO port used for USART1 TX.
 */
#define USART_TX_PORT GPIOA /**< Port A connected to USART1 TX */

/**
 * @brief GPIO pin used for USART1 TX.
 */
#define USART_TX_PIN GPIO_USART1_TX /**< PA9 connected to USART1 TX */

/**
 * @brief GPIO port used for USART1 RX.
 */
#define USART_RX_PORT GPIOA /**< Port A connected to USART1 RX */

/**
 * @brief GPIO pin used for USART1 RX.
 */
#define USART_RX_PIN GPIO_USART1_RX /**< PA10 connected to USART1 RX */

/**
 * @file tasks.c
 * @brief Implementation of tasks for temperature control, battery level monitoring, and door management.
 */

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

/**
 * @brief Stack size for the temperature control task.
 * 
 * Defines the total stack size used by the temperature control task, which includes the minimum 
 * stack size plus an additional 100 bytes.
 */
#define TEMPERATURE_CONTROL_STACK_SIZE (configMINIMAL_STACK_SIZE + 100) /**< Stack size for temperature control task */

/**
 * @brief Stack size for the battery level indicator task.
 * 
 * Defines the total stack size used by the battery level indicator task, which includes the minimum 
 * stack size plus an additional 100 bytes.
 */
#define BATTERY_LEVEL_INDICATOR_STACK_SIZE (configMINIMAL_STACK_SIZE + 100) /**< Stack size for battery level indicator task */

/**
 * @brief Stack size for the close door task.
 * 
 * Defines the total stack size used by the close door task, which includes the minimum stack size 
 * plus an additional 100 bytes.
 */
#define CLOSE_DOOR_STACK_SIZE (configMINIMAL_STACK_SIZE + 100) /**< Stack size for close door task */

/**
 * @brief Stack size for the open door task.
 * 
 * Defines the total stack size used by the open door task, which includes the minimum stack size 
 * plus an additional 100 bytes.
 */
#define OPEN_DOOR_STACK_SIZE (configMINIMAL_STACK_SIZE + 100) /**< Stack size for open door task */


/* Global variables */

/**
 * @brief Buffer for DMA transmission
 */
static char tx_buffer[TX_BUFFER_SIZE];  /**< Transmission buffer */

/**
 * @brief Duty cycle value for PWM control
 */
static uint32_t duty_cycle = 0; /**< PWM duty cycle */

/**
 * @brief Temperature value (from ADC)
 */
static uint32_t temp_value = 0; /**< Temperature value */

/**
 * @brief Edge direction for EXTI (interrupt) on pin 0
 */
static uint16_t exti_direction_0 = FALLING;

/**
 * @brief Edge direction for EXTI (interrupt) on pin 1
 */
static uint16_t exti_direction_1 = FALLING;

/**
 * @brief Edge direction for EXTI (interrupt) on pin 2
 */
static uint16_t exti_direction_2 = FALLING;

/**
 * @brief Edge direction for EXTI (interrupt) on pin 3
 */
static uint16_t exti_direction_3 = FALLING;

/**
 * @brief ADC value buffer for battery and temperature readings
 */
static int get_batt = 0; /**< Battery value flag */
static int get_temp = 0; /**< Temperature value flag */

/**
 * @brief ADC data buffer
 */
static uint16_t adc_data = 0; /**< ADC data buffer */

/**
 * @brief Door state: open or closed
 */
static door_state = OPEN; /**< Door state */

/**
 * @brief PWM value for fan control
 */
static pwm_fan = 0; /**< PWM value for fan control */

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
 * @brief Initializes the system clock to 72 MHz using an 8 MHz external crystal.
 */
void system_clock_setup(void)
{
    /* Configure the system clock to 72 MHz using PLL and 8 MHz HSE */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

/**
 * @brief Configures GPIO pins for motor control, sensors, alarms, and other components.
 * 
 * This function configures the following GPIO pins:
 * - Motor negative and positive control pins (MOTOR_NEG_PIN and MOTOR_POS_PIN)
 * - Onboard LED and fan control pins (LED_PIN and FAN_PIN)
 * - Alarm control pin (ALARM_PIN)
 * - Motion and fire sensor input pins (MOTION_SENSOR_PIN and FIRE_SENSOR_PIN)
 * - Button (switch) and override button input pins (SWITCH_PIN and OVERRIDE_PIN)
 * 
 * It also enables the necessary peripheral clocks for GPIOA, GPIOB, and GPIOC.
 * The pins are configured as input or output with appropriate settings (push-pull or pull-up/down).
 */
void gpio_setup(void)
{
    /* Enable GPIO clocks */
    rcc_periph_clock_enable(RCC_GPIOC); /**< Enable GPIOC peripheral clock */
    rcc_periph_clock_enable(RCC_GPIOA); /**< Enable GPIOA peripheral clock */
    rcc_periph_clock_enable(RCC_GPIOB); /**< Enable GPIOB peripheral clock */

    /* Configure GPIO pins for motor control */
    gpio_set_mode(MOTOR_NEG_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, MOTOR_NEG_PIN); /**< Set MOTOR_NEG_PIN as output with push-pull configuration */
    gpio_set_mode(MOTOR_POS_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, MOTOR_POS_PIN); /**< Set MOTOR_POS_PIN as output with push-pull configuration */

    /* Configure GPIO pins for LED and Fan */
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, LED_PIN); /**< Set LED_PIN as output with alternate function push-pull configuration */
    gpio_set_mode(FAN_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, FAN_PIN); /**< Set FAN_PIN as output with alternate function push-pull configuration */

    /* Configure GPIO pin for Alarm */
    gpio_set_mode(ALARM_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, ALARM_PIN); /**< Set ALARM_PIN as output with push-pull configuration */

    /* Configure GPIO pins for motion and fire sensors */
    gpio_set_mode(MOTION_SENSOR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, MOTION_SENSOR_PIN); /**< Set MOTION_SENSOR_PIN as input with pull-up/down configuration */
    gpio_set_mode(FIRE_SENSOR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, FIRE_SENSOR_PIN); /**< Set FIRE_SENSOR_PIN as input with pull-up/down configuration */

    /* Configure PA0 as input for switch with pull-up/down configuration */
    gpio_set_mode(SWITCH_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SWITCH_PIN); /**< Set SWITCH_PIN as input with pull-up/down configuration */

    /* Configure PA1 as input for override button with pull-up/down configuration */
    gpio_set_mode(OVERRIDE_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, OVERRIDE_PIN); /**< Set OVERRIDE_PIN as input with pull-up/down configuration */
}


/**
 * @brief Configures EXTI for PA0 to generate interrupts on both falling and rising edges.
 */
void exti_setup(void)
{
    /* Enable AFIO clock (for EXTI configuration) */
    rcc_periph_clock_enable(RCC_AFIO);

    /* Enable EXTI0 interrupt in the NVIC */
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    nvic_enable_irq(NVIC_EXTI1_IRQ);
    nvic_enable_irq(NVIC_EXTI2_IRQ);
    nvic_enable_irq(NVIC_EXTI3_IRQ);

    /* Configure EXTI0 (PA0) for falling edge initially */
    exti_select_source(EXTI0, SWITCH_PORT);        /* Set PA0 as the EXTI0 source */
    exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI0);                    /* Enable EXTI0 interrupt */

    /* Configure EXTI1 (PA1) for falling edge initially */
    exti_select_source(EXTI1, OVERRIDE_PORT);        /* Set PA1 as the EXTI1 source */
    exti_set_trigger(EXTI1, EXTI_TRIGGER_BOTH); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI1);                    /* Enable EXTI1 interrupt */

    /* Configure EXTI2 (PA2) for falling edge initially */
    exti_select_source(EXTI2, MOTION_SENSOR_PORT);        /* Set PA2 as the EXTI2 source */
    exti_set_trigger(EXTI2, EXTI_TRIGGER_BOTH); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI2);                    /* Enable EXTI2 interrupt */

    /* Configure EXTI3 (PA3) for risign edge initially */
    exti_select_source(EXTI3, FIRE_SENSOR_PORT);        /* Set PA3 as the EXTI3 source */
    exti_set_trigger(EXTI3, EXTI_TRIGGER_BOTH); /* Trigger interrupt on rising edge */
    exti_enable_request(EXTI3);                    /* Enable EXTI3 interrupt */
}

/**
 * @brief Configures the ADC for a specific channel.
 *
 * This function configures the ADC to work with a single channel, enables the clock, 
 * sets sample times, and starts the conversion process. It also enables DMA mode for the ADC.
 */
void configure_adc(void)
{
    /* Enable clock for ADC1 */
    rcc_periph_clock_enable(ADC_CLOCK_ENABLE);

    /* Turn off the ADC before configuring it */
    ADC1_CR2 &= ADC_DISABLE;

    /* Configure the regular sequence length to 1 (single channel) */
    ADC1_SQR1 &= ~(ADC_SEQ_LENGTH_MASK << ADC_SEQ_LENGTH_POS);  /* Clear sequence length bits */
    ADC1_SQR1 |= (ADC_SEQ_LENGTH_1_CHANNEL << ADC_SEQ_LENGTH_POS); /* Set sequence length to 1 channel */

    /* Configure the sample time for channel 4 */
    ADC1_SMPR2 &= ~(ADC_SAMPLE_TIME_MASK << ADC_CHANNEL_4_SAMPLE_POS); /* Clear sample time bits for channel 4 */
    ADC1_SMPR2 |= (ADC_SAMPLE_TIME_1_5_CYCLES << ADC_CHANNEL_4_SAMPLE_POS); /* Set sample time to 1.5 cycles for channel 4 */

    /* Configure the sample time for channel 5 */
    ADC1_SMPR2 &= ~(ADC_SAMPLE_TIME_MASK << ADC_CHANNEL_5_SAMPLE_POS); /* Clear sample time bits for channel 5 */
    ADC1_SMPR2 |= (ADC_SAMPLE_TIME_1_5_CYCLES << ADC_CHANNEL_5_SAMPLE_POS); /* Set sample time to 1.5 cycles for channel 5 */

    /* Start the regular conversion */
    ADC1_CR2 |= ADC_START_CONVERSION;

    /* Enable DMA mode for ADC */
    ADC1_CR2 |= ADC_ENABLE_DMA;
}

/**
 * @brief Changes the ADC sequence based on the active channel (battery or temperature)
 */
void change_adc_sequence(void)
{
    ADC1_CR2 &= ADC_DISABLE; /**< Disable ADC before changing the sequence */

    dma_disable_channel(DMA1, DMA_CHANNEL1); /**< Disable DMA channel */

    if(get_batt == TRUE){
        /* Configure channel 4 (battery) as the first channel in the sequence */
        ADC1_SQR3 &= ~ADC_SEQUENCE_CHANNEL_MASK; /**< Clear the bits for channel configuration */
        ADC1_SQR3 |= (ADC_CHANNEL_BATT << ADC_SEQUENCE_CHANNEL_POS); /**< Set channel 4 (battery) */
    }

    if(get_temp == TRUE){
        /* Configure channel 5 (temperature) as the first channel in the sequence */
        ADC1_SQR3 &= ~ADC_SEQUENCE_CHANNEL_MASK; /**< Clear the bits for channel configuration */
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
    if(xSemaphoreTake(xADC, portMAX_DELAY) == pdTRUE){
        get_batt = TRUE;           /**< Set the flag to read the battery */
        get_temp = FALSE;          /**< Clear the flag for temperature reading */
        change_adc_sequence();     /**< Change ADC sequence for battery reading */
        adc_start_conversion_direct(ADC1);  /**< Start ADC conversion for battery */
        xSemaphoreGive(xADC);      /**< Release the semaphore */
    }
    return((adc_data * PERCENTAGE) / ADC_MAX_VALUE); /**< Return the battery value as a percentage */
}

/**
 * @brief Reads the current temperature value from the ADC
 */
uint16_t get_temp_value(void)
{
    if(xSemaphoreTake(xADC, portMAX_DELAY) == pdTRUE){
        get_batt = FALSE;          /**< Clear the flag for battery reading */
        get_temp = TRUE;           /**< Set the flag to read the temperature */
        change_adc_sequence();     /**< Change ADC sequence for temperature reading */
        adc_start_conversion_direct(ADC1);  /**< Start ADC conversion for temperature */
        xSemaphoreGive(xADC);      /**< Release the semaphore */
    }
    return ((adc_data * TEMP_CONVERSION_FACTOR) / ADC_MAX_VALUE); /**< Return the temperature value in Celsius */
}

/**
 * @brief Configures USART1 with specified settings.
 * 
 * This function sets up USART1 with a baud rate of 9600, 8 data bits, 
 * 1 stop bit, and no parity. It also enables GPIOA for USART TX and RX, 
 * configures the pins for alternate function push-pull, and enables 
 * DMA for TX.
 * 
 * @details
 * - Enables the peripheral clocks for USART1 and GPIOA.
 * - Configures GPIOA pins PA9 and PA10 for TX and RX respectively.
 * - Sets up USART1 with the specified parameters and enables the peripheral.
 */
void configure_usart(void) {
    rcc_periph_clock_enable(RCC_USART1);        /* Enable USART1 clock */
    rcc_periph_clock_enable(RCC_GPIOA);         /* Enable GPIOA clock for USART */

    gpio_set_mode(
        USART_TX_PORT,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        USART_TX_PIN
    ); /* Configure TX pin */
    gpio_set_mode(
        USART_RX_PORT,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        USART_RX_PIN
    ); /* Configure RX pin as floating input */

    usart_set_baudrate(USART1, UART_BAUD_RATE); /* Set USART1 baud rate */
    usart_set_databits(USART1, UART_DATA_BITS); /* Set USART1 data bits */
    usart_set_stopbits(USART1, USART_STOPBITS_1); /* Set USART1 stop bits */
    usart_set_mode(USART1, USART_MODE_TX); /* Set USART1 mode to transmit only */
    usart_set_parity(USART1, USART_PARITY_NONE); /* Set USART1 parity to none */
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE); /* Disable USART1 flow control */

    usart_enable_tx_dma(USART1); /* Enable USART1 TX DMA */

    usart_enable(USART1); /* Enable USART1 peripheral */
}

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
void dma_setup(void) {
    // Enable the clock for DMA1
    rcc_periph_clock_enable(RCC_DMA1);

    // --- DMA1 Channel 1 Configuration (ADC) ---
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1)); /* ADC1 data register as the source. */
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)&adc_data); /* Memory buffer as destination. */
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT); /* 16-bit data size for peripheral. */
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT); /* 16-bit data size for memory. */
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1); /* Transfer 1 data unit per cycle. */
    dma_disable_memory_increment_mode(DMA1, DMA_CHANNEL1); /* Memory address is fixed. */
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1); /* Peripheral address is fixed. */
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH); /* Set high priority for ADC transfers. */
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1); /* Enable circular mode for continuous transfers. */
    dma_enable_channel(DMA1, DMA_CHANNEL1); /* Activate DMA1 Channel 1. */

    // --- DMA1 Channel 4 Configuration (USART1 TX) ---
    dma_disable_channel(DMA1, DMA_CHANNEL4); /* Ensure Channel 4 is disabled during setup. */
    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR); /* USART1 data register as the destination. */
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)tx_buffer); /* TX buffer as the data source. */
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT); /* 8-bit data size for peripheral. */
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT); /* 8-bit data size for memory. */
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4); /* Memory as the source of data. */
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4); /* Enable memory address increment. */
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_MEDIUM); /* Set medium priority for USART1 transfers. */
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, TX_BUFFER_SIZE); /* Number of bytes to transfer. */
    dma_enable_channel(DMA1, DMA_CHANNEL4); /* Activate DMA1 Channel 4. */

    // Enable transfer complete interrupt for DMA1 Channel 4
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ); /* Enable NVIC interrupt for DMA1 Channel 4. */
}

/**
 * @brief DMA1 Channel 4 interrupt service routine.
 *
 * Handles the transfer complete interrupt for USART1 TX DMA transfer.
 * - Clears the interrupt flag.
 * - Disables the DMA channel to prevent further transfers.
 * - Gives the xUART semaphore to indicate the transfer is complete.
 */
void dma1_channel4_isr(void) {
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF); /* Clear transfer complete flag. */
        dma_disable_channel(DMA1, DMA_CHANNEL4); /* Disable DMA1 Channel 4. */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;  /* Task notification not required. */
        xSemaphoreGiveFromISR(xUART, &xHigherPriorityTaskWoken); /* Give the UART semaphore. */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); /* Switch to higher priority task if required. */
    }
}

/**
 * @brief Configures TIM4 to generate PWM signals.
 *
 * This function sets up TIM4 for PWM generation on channels 3 and 4:
 * - **Frequency**: Configured to 20 kHz.
 * - **Mode**: Center-aligned mode with counting direction up.
 * - **Channels**: 
 *   - Channel 3 and 4 are set to PWM mode 2.
 * - **Output**: Enables high polarity output for both channels.
 * - **Break Functionality**: Enables main output break for safety.
 */
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
void usart_send_string(const char *str, ...) {
    if (!(xSemaphoreTake(xUART, portMAX_DELAY) == pdTRUE)) { /* Check if the UART semaphore is available */
        return;
    }

    char formatted_buffer[TX_BUFFER_SIZE]; /* Buffer for formatted string */
    va_list args; /* Variable argument list */
    va_start(args, str); /* Initialize variable arguments */
    int len = vsnprintf(formatted_buffer, sizeof(formatted_buffer), str, args); /* Format the string */
    va_end(args); /* End variable arguments */

    if (len < 0 || len >= TX_BUFFER_SIZE) { /* Check for errors in formatting */
        return;
    }

    memcpy(tx_buffer, formatted_buffer, len); /* Copy the formatted string to the TX buffer */

    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)tx_buffer); /* Set the memory address for DMA transfer */
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, len); /* Set the number of bytes to transfer */
 
    dma_enable_channel(DMA1, DMA_CHANNEL4); /* Enable DMA channel 4 */
}

/**
 * @brief Closes the door by activating the motor in the closing direction.
 *
 * This function sets the motor control pins to rotate the motor for closing the door.
 * Additionally, it sends a status message via USART to indicate the operation.
 */
void close_door(void) {
    gpio_set(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /**< Activate motor for closing. */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /**< Deactivate motor in opening direction. */
    //usart_send_string("Cerrando puerta\r\n");
}

/**
 * @brief Opens the door by activating the motor in the opening direction.
 *
 * This function sets the motor control pins to rotate the motor for opening the door.
 * Additionally, it sends a status message via USART to indicate the operation.
 */
void open_door(void) {
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /**< Deactivate motor in closing direction. */
    gpio_set(MOTOR_POS_PORT, MOTOR_POS_PIN); /**< Activate motor for opening. */
    //usart_send_string("Abriendo puerta\r\n");
}

/**
 * @brief Stops the motor by disabling both motor control pins.
 *
 * This function clears the motor control pins to stop any ongoing motor operation.
 */
void stop_motor(void) {
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /**< Disable motor in closing direction. */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /**< Disable motor in opening direction. */
}

/**
 * @brief Task to control the temperature and manage the fan speed via PWM.
 *
 * This task continuously monitors the temperature, adjusts the PWM duty cycle 
 * for the fan, and sends notifications to close the door if the temperature is below 
 * a specified threshold.
 *
 * @param pvParameters Unused parameter for FreeRTOS task compatibility.
 */
void temperature_control_task(void *pvParameters __attribute__((unused)))
{
    while(TRUE)    
    {
        temp_value = get_temp_value();

        if(temp_value < TEMPERATURE_THRESHOLD)
        {
            pwm_fan = TEMPERATURE_ZERO; /* Turn off fan*/
            /* Notify to close the door */
            xTaskNotifyGive(closeDoorTaskHandle);
        }
        else{
            pwm_fan = temp_value; /* Set fan speed based on temperature */
        }

        usart_send_string("Temperatura actual: %d°C\r\n", temp_value); /* Send temperature via serial */
        timer_set_oc_value(TIM4, TIM_OC4, TIMER_PERIOD - (TIMER_PERIOD * temp_value) / PERCENTAGE); /* Set PWM duty cycle */

        vTaskDelay(TWO_MINUTES_DELAY / portTICK_PERIOD_MS); /* Wait for two minutes between readings */
    }
}

/**
 * @brief Task to monitor and indicate battery level via PWM.
 *
 * This task reads the battery level, calculates the corresponding PWM duty cycle, 
 * and sends the battery percentage through the serial port.
 *
 * @param pvParameters Unused parameter for FreeRTOS task compatibility.
 */
void battery_level_indicator_task(void *pvParameters __attribute__((unused)))
{
    while(TRUE)
    {
        duty_cycle = get_battery_value(); /* Get battery percentage */
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
 *
 * @param pvParameters Parameter for FreeRTOS task compatibility.
 */
void close_door_task(void *pvParameters)
{
    while (TRUE)
    {
        /* Wait for a notification */
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0){
            if(xSemaphoreTake(xDoor, portMAX_DELAY) == pdTRUE){
                if(door_state == OPEN){
                    door_state = CLOSED; /* Set door state to closed */
                    close_door(); /* Close the door */
                    vTaskDelay(HALF_MINUTE_DELAY / portTICK_PERIOD_MS); /* Wait for door to close */
                    stop_motor(); /* Stop the motor */
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
 *
 * @param pvParameters Parameter for FreeRTOS task compatibility.
 */
void open_door_task(void *pvParameters)
{
    while (TRUE)
    {
        /* Wait for a notification */
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0){
            if(xSemaphoreTake(xDoor, portMAX_DELAY) == pdTRUE){
                if(door_state == CLOSED){
                    door_state = OPEN; /* Set door state to open */
                    open_door(); /* Open the door */
                    vTaskDelay(HALF_MINUTE_DELAY / portTICK_PERIOD_MS); /* Wait for door to open */
                    stop_motor(); /* Stop the motor */
                }
                xSemaphoreGive(xDoor); /* Release the semaphore */
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief ISR for EXTI0 interrupt (door control based on edge detection).
 *
 * This ISR toggles between opening and closing the door based on the 
 * edge detection state (falling or rising). It notifies the appropriate 
 * FreeRTOS task and switches the EXTI trigger to the opposite edge.
 */
void exti0_isr(void)
{
    /* Clear the EXTI0 interrupt flag */
    exti_reset_request(EXTI0);

    /* Check if the interrupt is due to falling edge */
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
 * Toggles between enabling or disabling manual controls based on 
 * the edge detection state (falling or rising). This is communicated 
 * via USART messages and updates the NVIC interrupt configuration.
 */
void exti1_isr(void)
{
    /* Clear the EXTI1 interrupt flag */
    exti_reset_request(EXTI1);

    /* Check if the interrupt is due to falling edge */
    if (!(gpio_get(OVERRIDE_PORT, OVERRIDE_PIN)))
    {
        /* Disable EXTI0 interrupt and enable EXTI2, EXTI3 */
        nvic_disable_irq(NVIC_EXTI0_IRQ);
        nvic_enable_irq(NVIC_EXTI2_IRQ);
        nvic_enable_irq(NVIC_EXTI3_IRQ);

        /* Notify manual controls deactivation */
        usart_send_string("Controles manuales desactivados\r\n");
    }
    else
    {
        /* Enable EXTI0 interrupt and disable EXTI2, EXTI3 */
        nvic_enable_irq(NVIC_EXTI0_IRQ);
        nvic_disable_irq(NVIC_EXTI2_IRQ);
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

    /* Check if the interrupt is due to falling edge */
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
 * to open or close the door. It also triggers the alarm, sending a message 
 * through USART to indicate the fire status and controls the alarm LED.
 */
void exti3_isr(void)
{
    /* Clear the EXTI3 interrupt flag */
    exti_reset_request(EXTI3);

    /* Check if the interrupt is due to falling edge */
    if(!gpio_get(FIRE_SENSOR_PORT, FIRE_SENSOR_PIN))
    {
        /* Deactivate the alarm */
        gpio_set(ALARM_PORT, ALARM_PIN); /* Turn off the alarm on rising edge */

        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        /* Switch to rising edge detection */
        //exti_direction_3 = RISING;
        //exti_set_trigger(EXTI3, EXTI_TRIGGER_RISING);

        /* Send fire detection messages */
        //usart_send_string("Fuego controlado\r\n");
        //usart_send_string("Activando alarma...\r\n");
    }
    else
    {
        /* Reactivate the alarm */
        gpio_clear(ALARM_PORT, ALARM_PIN);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        /* Switch to falling edge detection */
        //exti_direction_3 = FALLING;
        //exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING);

        /* Send fire controlled messages */
        //usart_send_string("Fuego detectado\n");
        //usart_send_string("Desactivando alarma\r\n");
    }
}

/**
 * @brief Application hook function for stack overflow handling.
 * 
 * This function is called when a stack overflow occurs in any task. It 
 * enters an infinite loop, halting further execution. It is used to 
 * debug stack overflow issues.
 * 
 * @param pxTask Handle to the task that caused the overflow (unused).
 * @param pcTaskName Name of the task that caused the overflow (unused).
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
    while(TRUE);  /**< Infinite loop to halt the system on stack overflow */
}

/**
 * @brief Configures binary semaphores for ADC and door control.
 * 
 * This function creates two binary semaphores used for synchronization:
 * one for controlling ADC access (`xADC`) and one for controlling door 
 * operations (`xDoor`). If the semaphores cannot be created, it enters 
 * an infinite loop and sends an error message over USART.
 */
void config_semaphore(void){
    // Create a binary semaphore for ADC access
    xADC = xSemaphoreCreateBinary();
    if(xADC == NULL){
        while(TRUE){  /**< Infinite loop in case of semaphore creation failure */
            usart_send_string("ERROR");  /**< Send error message if semaphore creation fails */
        }
    }
    xSemaphoreGive(xADC);  /**< Release the semaphore to allow ADC access */

    // Create a binary semaphore for door control
    xDoor = xSemaphoreCreateBinary();
    if(xDoor == NULL){
        while(TRUE){  /**< Infinite loop in case of semaphore creation failure */
            usart_send_string("ERROR");  /**< Send error message if semaphore creation fails */
        }
    }
    xSemaphoreGive(xDoor);  /**< Release the semaphore to allow door control */

    // Create a binary semaphore for UART access
    xUART = xSemaphoreCreateBinary(); 
    if(xUART == NULL){
        while(TRUE){  /**< Infinite loop in case of semaphore creation failure */
            usart_send_string("ERROR");  /**< Send error message if semaphore creation fails */
        }
    }
    xSemaphoreGive(xUART);  /**< Release the semaphore to allow UART access */
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
    xTaskCreate(temperature_control_task, TEMPERATURE_CONTROL_TASK_NAME, TEMPERATURE_CONTROL_STACK_SIZE, NULL, TEMPERATURE_CONTROL_PRIORITY, NULL);
    xTaskCreate(battery_level_indicator_task, BATTERY_LEVEL_INDICATOR_TASK_NAME, BATTERY_LEVEL_INDICATOR_STACK_SIZE, NULL, BATTERY_CONTROL_PRIORITY, NULL);
    xTaskCreate(close_door_task, CLOSE_DOOR_TASK_NAME, CLOSE_DOOR_STACK_SIZE, NULL, CLOSE_DOOR_PRIORITY, &closeDoorTaskHandle);
    xTaskCreate(open_door_task, OPEN_DOOR_TASK_NAME, OPEN_DOOR_STACK_SIZE, NULL, OPEN_DOOR_PRIORITY, &openDoorTaskHandle);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Main loop (in this case, the program relies on interrupts for operation)
    while (TRUE)
    {  
        // The main loop does nothing since tasks and interrupts handle the operation
    }

    return 0; /**< Return statement (will never be reached due to scheduler) */
}
