/*
 * @file main.c
 * @brief Toggles the LED on PC13 using a SysTick timer and EXTI0 interrupt.
 * Enable GPIO interrupt on PA0 to toggle the LED state on both rising and falling edges.
 * The onboard LED (connected to PC13 on STM32F103C8T6 Blue Pill) will blink using SysTick and toggle using EXTI0.
 * This file is based on examples from the libopencm3 project.
 */

/**
 * @file main.c
 * @brief Control system for various components (motor, sensors, door, etc.) using FreeRTOS and DMA.
 *
 * This file implements the control system that uses DMA for USART communication, 
 * FreeRTOS tasks for controlling various components like a motor, door, and sensors.
 * The system uses ADC to monitor battery levels and temperature, and EXTI interrupts 
 * for detecting button presses and sensor inputs.
 *
 * @author [Your Name]
 * @date [Current Date]
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

/** 
 * @brief Delay time for one minute in milliseconds
 */
#define ONE_MINUTE_DELAY 60000 /**< 60000 ms */

/* Task names */

/** 
 * @brief Task name for temperature control
 */
#define TEMPERATURE_CONTROL_TASK_NAME       "temperature_control"

/** 
 * @brief Task name for battery control
 */
#define BATTERY_CONTROL_TASK_NAME           "battery_control"

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
#define TEMP_CONVERSION_FACTOR      30                /**< Conversion factor for temperature calculation */

  

/* Global variables */

/**
 * @brief Buffer for DMA transmission
 */
static char tx_buffer[TX_BUFFER_SIZE];  /**< Transmission buffer */

/**
 * @brief Flag to indicate if DMA is busy with a transmission
 */
static volatile uint8_t dma_tx_busy = 0;  /**< DMA busy flag */

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

/* FreeRTOS Semaphore Handles */

/**
 * @brief Semaphore handle for ADC synchronization
 */
SemaphoreHandle_t xADC = NULL; /**< Semaphore for ADC synchronization */

/**
 * @brief Semaphore handle for door synchronization
 */
SemaphoreHandle_t xDoor = NULL; /**< Semaphore for door synchronization */

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
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI0);                    /* Enable EXTI0 interrupt */

    /* Configure EXTI1 (PA1) for falling edge initially */
    exti_select_source(EXTI1, SWITCH_PORT);        /* Set PA1 as the EXTI1 source */
    exti_set_trigger(EXTI1, EXTI_TRIGGER_FALLING); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI1);                    /* Enable EXTI1 interrupt */

    /* Configure EXTI2 (PA2) for falling edge initially */
    exti_select_source(EXTI2, SWITCH_PORT);        /* Set PA2 as the EXTI2 source */
    exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI2);                    /* Enable EXTI2 interrupt */

    /* Configure EXTI3 (PA3) for falling edge initially */
    exti_select_source(EXTI3, SWITCH_PORT);        /* Set PA3 as the EXTI3 source */
    exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING); /* Trigger interrupt on falling edge */
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


void configure_usart(void) {
    rcc_periph_clock_enable(RCC_USART1);        // Habilita USART1
    rcc_periph_clock_enable(RCC_GPIOA);         // Habilita GPIOA para USART
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // TX
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX); // RX como entrada flotante
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable_tx_dma(USART1);
    usart_enable(USART1);
}

void dma_setup(void)
{
    
    // Habilita el reloj para el DMA1 y GPIOC
    rcc_periph_clock_enable(RCC_DMA1);
    
    // Configura el dma para transferir datos 
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    // Origen de los datos a transferir:
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, &ADC_DR(ADC1));
    // Dirección de datos destino. El ODR (Output Data Register) del GPIOA:
    dma_set_memory_address(DMA1, DMA_CHANNEL1, &adc_data);
    // Tamaño del dato a leer
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    // Tamaño del dato a escribir
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);


    // Cantidad de datos a transferir:
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1);

    // Se incrementa automaticamente la posición en memoria:
    dma_disable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    // La dirección destino se mantiene fija:
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
    // Se establece la prioridad del canal 7 del DMA1 como alta:
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);
    //Se habilita el modo circular para que la transferencia se repita indefinidamente
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);

    dma_enable_channel(DMA1, DMA_CHANNEL1);

    dma_disable_channel(DMA1, DMA_CHANNEL4); // Canal asociado al USART1 TX

    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR);
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)tx_buffer);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_MEDIUM);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, TX_BUFFER_SIZE);

    dma_enable_channel(DMA1, DMA_CHANNEL4);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
}

void dma1_channel4_isr(void) {
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF)) {
        // Limpia la bandera de interrupción
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF);
        dma_disable_channel(DMA1, DMA_CHANNEL4);
        dma_tx_busy = 0;
    }
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

void usart_send_string(const char *str, ...) {
    if (dma_tx_busy) {
        // El DMA está ocupado, no podemos enviar
        return;
    }

    char formatted_buffer[TX_BUFFER_SIZE]; // Buffer temporal
    va_list args;
    va_start(args, str);
    int len = vsnprintf(formatted_buffer, sizeof(formatted_buffer), str, args);
    va_end(args);

    if (len < 0 || len >= TX_BUFFER_SIZE) {
        // Error o cadena demasiado grande
        return;
    }

    // Copiar al buffer DMA
    memcpy(tx_buffer, formatted_buffer, len);

    dma_tx_busy = 1; // Marca el DMA como ocupado

    // Configurar DMA
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)tx_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, len);

    // Iniciar la transferencia
    dma_enable_channel(DMA1, DMA_CHANNEL4);
}

// Función de interrupción o callback al terminar la transmisión por DMA
void dma_tx_complete_callback(void) {
    dma_tx_busy = 0; // Liberar la bandera de ocupación del DMA
}

void close_door(void){
    gpio_set(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Turn on LED on falling edge */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Turn on LED on falling edge */
    usart_send_string("Cerrando puerta\r\n");
}

void open_door(void){
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Turn off LED on rising edge */
    gpio_set(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Turn off LED on rising edge */
    usart_send_string("Abriendo puerta\r\n");
}

void stop_motor(void){
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Turn off LED on rising edge */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Turn off LED on rising edge */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void temperature_control_task(void *pvParameters __attribute__((unused)))
{
    while(TRUE)    
    {
        temp_value = get_temp_value();

        if(temp_value < 24)
        {
            temp_value = 0; // Turn off fan
            // Notify close_door_task
            xTaskNotifyGive(closeDoorTaskHandle);
        }


        usart_send_string("Temperatura actual: %d°C\r\n", temp_value); // Envía el valor del ADC por el puerto serial
        timer_set_oc_value(TIM4, TIM_OC4, TIMER_PERIOD - (TIMER_PERIOD * temp_value) / 30); // Configura el duty cycle del PWM

        vTaskDelay(4000 / portTICK_PERIOD_MS); // Wait one minute between temperature readings
    }
}

void battery_level_indicator_task(void *pvParameters __attribute__((unused)))
{
    while(TRUE)
    {
        duty_cycle = get_battery_value(); // Lee el valor del ADC
        usart_send_string("Porcentaje de bateria: %d%%\r\n", duty_cycle); // Envía el valor del ADC por el puerto serial
        uint32_t pwm_value = TIMER_PERIOD - (TIMER_PERIOD * duty_cycle) / PERCENTAGE;
        timer_set_oc_value(TIM4, TIM_OC3, pwm_value); // Configura el duty cycle del PWM
        
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Wait one minute between temperature readings
    }    
}


void close_door_task(void *pvParameters)
{
    while (TRUE)
    {
        // Wait for any notification
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)>0){
            if(xSemaphoreTake(xDoor, portMAX_DELAY) == pdTRUE){
                if(door_state == OPEN){
                    door_state = CLOSED;
                    close_door();
                    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for the door to close
                    stop_motor();
                }
                xSemaphoreGive(xDoor);
            }
        }
    }
}

void open_door_task(void *pvParameters)
{
    while (TRUE)
    {
        // Wait for any notification
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)>0){
            if(xSemaphoreTake(xDoor, portMAX_DELAY) == pdTRUE){
                if(door_state == CLOSED){
                    door_state = OPEN;
                    open_door();
                    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for the door to open
                    stop_motor();
                }
                xSemaphoreGive(xDoor);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief EXTI0 interrupt handler for button press on PA0.
 * Toggles the LED state when the button is pressed (detects both falling and rising edges).
 */
void exti0_isr(void)
{
    /* Clear the EXTI0 interrupt flag */
    exti_reset_request(EXTI0);

    /* Toggle the LED and switch EXTI edge detection */
    if (exti_direction_0 == FALLING)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_0 = RISING;     /* Switch to rising edge detection */
        exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    }
    else
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_0 = FALLING;      /* Switch to falling edge detection */
        exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    }
}

/**
 * @brief EXTI0 interrupt handler for button press on PA0.
 * Toggles the LED state when the button is pressed (detects both falling and rising edges).
 */
void exti1_isr(void)
{
    /* Clear the EXTI1 interrupt flag */
    exti_reset_request(EXTI1);

    /* Toggle the LED and switch EXTI edge detection */
    if (exti_direction_1 == FALLING)
    {

        nvic_disable_irq(NVIC_EXTI0_IRQ);
        nvic_enable_irq(NVIC_EXTI2_IRQ);
        nvic_enable_irq(NVIC_EXTI3_IRQ);
        usart_send_string("Controles manuales desactivados\r\n");   
        exti_direction_1 = RISING;     /* Switch to rising edge detection */
        exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    }
    else
    {
        nvic_enable_irq(NVIC_EXTI0_IRQ);
        nvic_disable_irq(NVIC_EXTI2_IRQ);
        nvic_disable_irq(NVIC_EXTI3_IRQ);
        usart_send_string("Controles manuales activados\r\n");
        exti_direction_1 = FALLING;      /* Switch to falling edge detection */
        exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    }
}

/**
 * @brief EXTI0 interrupt handler for button press on PA0.
 * Toggles the LED state when the button is pressed (detects both falling and rising edges).
 */
void exti2_isr(void)
{
    /* Clear the EXTI2 interrupt flag */
    exti_reset_request(EXTI2);

    if (exti_direction_2 == FALLING)
    {
        usart_send_string("Zona asegurada\r\n");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        
        exti_direction_2 = RISING;     /* Switch to rising edge detection */
        exti_set_trigger(EXTI2, EXTI_TRIGGER_RISING);
        }
    else
    {
        usart_send_string("Zombies cerca\r\n");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_2 = FALLING;      /* Switch to falling edge detection */
        exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);
    }
}

void exti3_isr(void)
{
    /* Clear the EXTI3 interrupt flag */
    exti_reset_request(EXTI3);

    if (exti_direction_3 == FALLING)
    {

        gpio_clear(ALARM_PORT, ALARM_PIN); /* Turn on LED on falling edge */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_3 = RISING;     /* Switch to rising edge detection */
        exti_set_trigger(EXTI3, EXTI_TRIGGER_RISING);
        usart_send_string("Fuego detectado!!!!\r\n");
        usart_send_string("Activando alarma...\r\n");
    }
    else
    {
        gpio_set(ALARM_PORT, ALARM_PIN); /* Turn off LED on rising edge */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_3 = FALLING;      /* Switch to falling edge detection */
        exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING);
        usart_send_string("Fuego controlado\r\n");
        usart_send_string("Desactivando alarma\r\n");
    }
}



void vApplicationStackOverflowHook(TaskHandle_t pxTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
    while(1);
}

void config_semaphore(void){
    xADC = xSemaphoreCreateBinary();
    if(xADC == NULL){
        while(1){
            usart_send_string("ERROR");
        }
    }
    xSemaphoreGive(xADC);

    xDoor = xSemaphoreCreateBinary();
    if(xDoor == NULL){
        while(1){
            usart_send_string("ERROR");
        }
    }
    xSemaphoreGive(xDoor);
}

/**
 * @brief Main function.
 * Initializes the system clock, GPIO, SysTick, and EXTI, then enters an infinite loop.
 */
int main(void)
{
    system_clock_setup(); /* Configure system clock */
    gpio_setup();         /* Configure GPIO pins */
    exti_setup();         /* Configure EXTI for button press detection */
    configure_adc();      // Configura el ADC
    configure_usart();    // Configura el puerto serial
    config_pwm();         // Configura el PWM
    dma_setup();          // Configura el DMA

    config_semaphore();


    nvic_set_priority(NVIC_EXTI0_IRQ, 2); /* Set EXTI0 interrupt priority to 0 (highest) */
    nvic_set_priority(NVIC_EXTI1_IRQ, 0); /* Set EXTI1 interrupt priority to 0 (highest) */

    // Task creation
    xTaskCreate(temperature_control_task, TEMPERATURE_CONTROL_TASK_NAME, configMINIMAL_STACK_SIZE + 100, NULL, TEMPERATURE_CONTROL_PRIORITY, NULL);
    xTaskCreate(battery_level_indicator_task, BATTERY_CONTROL_TASK_NAME, configMINIMAL_STACK_SIZE + 100, NULL, BATTERY_CONTROL_PRIORITY, NULL);
    xTaskCreate(close_door_task, CLOSE_DOOR_TASK_NAME, configMINIMAL_STACK_SIZE + 100, NULL, CLOSE_DOOR_PRIORITY, &closeDoorTaskHandle);
    xTaskCreate(open_door_task, OPEN_DOOR_TASK_NAME, configMINIMAL_STACK_SIZE + 100, NULL, OPEN_DOOR_PRIORITY, &openDoorTaskHandle);

    vTaskStartScheduler();

    /* Main loop (the program relies on interrupts for operation) */
    while (TRUE)
    {  
    
    }

    return 0;
}
