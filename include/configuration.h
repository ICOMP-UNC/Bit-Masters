#pragma once

// libopencm3
#include <libopencm3/cm3/nvic.h> /**< NVIC (Nested Vectored Interrupt Controller) library for managing interrupt priority and enabling interrupts. */
#include <libopencm3/cm3/systick.h> /**< Systick (System Tick Timer) library for setting up and managing system timer interrupts. */
#include <libopencm3/stm32/adc.h> /**< ADC (Analog-to-Digital Converter) library for configuring and using ADCs to sample analog signals. */
#include <libopencm3/stm32/dma.h> /**< DMA (Direct Memory Access) library for managing data transfers between peripherals and memory. */
#include <libopencm3/stm32/exti.h> /**< EXTI (External Interrupt) library for handling external interrupt lines and events. */
#include <libopencm3/stm32/gpio.h> /**< GPIO (General Purpose Input/Output) library for configuring and controlling the GPIO pins. */
#include <libopencm3/stm32/rcc.h> /**< RCC (Reset and Clock Control) library for managing system clocks and peripheral clocks. */
#include <libopencm3/stm32/timer.h> /**< Timer library for configuring and controlling timers for time-based events or PWM generation. */
#include <libopencm3/stm32/usart.h> /**< USART (Universal Synchronous Asynchronous Receiver-Transmitter) library for serial communication. */

#include <stdarg.h> /**< Standard library for handling variable arguments in functions (e.g., va_list, va_start, va_end). */
#include <stdio.h> /**< Standard Input/Output library for functions like printf, scanf, etc. */

// FreeRTOS headers
#include "FreeRTOS.h" /**< FreeRTOS core header file, provides the main API for task management and kernel functions. */
#include "semphr.h"   /**< FreeRTOS header for semaphore management, used for synchronization between tasks. */
#include "task.h"     /**< FreeRTOS header for task management, including task creation, deletion, and scheduling. */

/* Constants */

/**
 * @brief Define for logical TRUE
 */
#define TRUE 1

/**
 * @brief Define for logical FALSE
 */
#define FALSE 0

/**
 * @brief Define for falling edge detection
 */
#define FALLING 0

/**
 * @brief Define for rising edge detection
 */
#define RISING 1

/* Pin Definitions */

/**
 * @brief Port and pin configuration for motor negative terminal
 */
#define MOTOR_NEG_PORT GPIOA /**< Port A connected to motor negative terminal */
#define MOTOR_NEG_PIN  GPIO7 /**< PA7 connected to motor negative terminal */

/**
 * @brief Port and pin configuration for motor positive terminal
 */
#define MOTOR_POS_PORT GPIOA /**< Port A connected to motor positive terminal */
#define MOTOR_POS_PIN  GPIO6 /**< PA6 connected to motor positive terminal */

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

/* Task names */

/**
 * @brief Task name for temperature control
 */
#define TEMPERATURE_CONTROL_TASK_NAME "temperature_control"

/**
 * @brief Task name for battery control
 */
#define BATTERY_LEVEL_INDICATOR_TASK_NAME "battery_control"

/**
 * @brief Task name for door control to close
 */
#define CLOSE_DOOR_TASK_NAME "close_door"

/**
 * @brief Task name for door control to open
 */
#define OPEN_DOOR_TASK_NAME "open_door"

/* Task priorities */

/**
 * @brief Priority for temperature control task
 */
#define TEMPERATURE_CONTROL_PRIORITY tskIDLE_PRIORITY + 2

/**
 * @brief Priority for battery control task
 */
#define BATTERY_CONTROL_PRIORITY tskIDLE_PRIORITY + 1

/**
 * @brief Priority for door close task
 */
#define CLOSE_DOOR_PRIORITY tskIDLE_PRIORITY + 2

/**
 * @brief Priority for door open task
 */
#define OPEN_DOOR_PRIORITY tskIDLE_PRIORITY + 2

/**
 * @brief Stack size for the temperature control task.
 *
 * Defines the total stack size used by the temperature control task, which includes the minimum
 * stack size plus an additional 100 bytes.
 */
#define TEMPERATURE_CONTROL_STACK_SIZE (configMINIMAL_STACK_SIZE + 100) /**< Stack size for temperature control task   \
                                                                         */

/**
 * @brief Stack size for the battery level indicator task.
 *
 * Defines the total stack size used by the battery level indicator task, which includes the minimum
 * stack size plus an additional 100 bytes.
 */
#define BATTERY_LEVEL_INDICATOR_STACK_SIZE                                                                             \
    (configMINIMAL_STACK_SIZE + 100) /**< Stack size for battery level indicator task */

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

/**
 * @brief ADC clock enable macro for ADC1
 */
#define ADC_CLOCK_ENABLE RCC_ADC1 /**< Enable the clock for ADC1 */

/**
 * @brief ADC disable macro for ADC1
 */
#define ADC_DISABLE (~ADC_CR2_ADON) /**< Clear the ADON bit to disable the ADC */

/**
 * @brief Mask for the sequence length bits in ADC1_SQR1 register
 */
#define ADC_SEQ_LENGTH_MASK 0xF /**< Mask for the sequence length bits */

/**
 * @brief Position of the sequence length bits in ADC1_SQR1 register
 */
#define ADC_SEQ_LENGTH_POS 20 /**< Position of the sequence length bits */

/**
 * @brief Sequence length set to 1 channel in ADC configuration
 */
#define ADC_SEQ_LENGTH_1_CHANNEL 0x0 /**< Set the sequence length to 1 channel */

/**
 * @brief Mask for the sample time bits in ADC1_SMPR2 register
 */
#define ADC_SAMPLE_TIME_MASK 0x7 /**< Mask for the sample time bits */

/**
 * @brief Position of the sample time bits for channel 4 in ADC1_SMPR2 register
 */
#define ADC_CHANNEL_4_SAMPLE_POS 12 /**< Position of the sample time bits for channel 4 */

/**
 * @brief Position of the sample time bits for channel 5 in ADC1_SMPR2 register
 */
#define ADC_CHANNEL_5_SAMPLE_POS 15 /**< Position of the sample time bits for channel 5 */

/**
 * @brief Sample time of 1.5 cycles for ADC channel configuration
 */
#define ADC_SAMPLE_TIME_1_5_CYCLES ADC_SMPR_SMP_1DOT5CYC /**< Set sample time to 1.5 cycles */

/**
 * @brief ADC conversion start macro for regular conversion
 */
#define ADC_START_CONVERSION ADC_CR2_SWSTART /**< Set the SWSTART bit to begin conversion */

/**
 * @brief ADC DMA enable macro to enable DMA for ADC
 */
#define ADC_ENABLE_DMA ADC_CR2_DMA /**< Enable DMA mode for ADC */

/**
 * @brief Define for enabling the ADC (ADC1)
 */
#define ADC_ENABLE ADC_CR2_ADON /**< Set the ADON bit to enable ADC1 */

/**
 * @brief Define for disabling the ADC (ADC1)
 */
#define ADC_DISABLE (~ADC_CR2_ADON) /**< Clear the ADON bit to disable ADC1 */

/**
 * @brief Define for the ADC sequence channel mask in the ADC1_SQR3 register
 */
#define ADC_SEQUENCE_CHANNEL_MASK 0x1F /**< Mask for the sequence channel bits */

/**
 * @brief Define for the position of the sequence channel in the ADC1_SQR3 register
 */
#define ADC_SEQUENCE_CHANNEL_POS 0 /**< Position of the channel in the sequence */

/**
 * @brief Define for the ADC sequence channel mask in the ADC1_SQR3 register
 */
#define ADC_SEQUENCE_CHANNEL_MASK 0x1F /**< Mask for the sequence channel bits */

/**
 * @brief Define for the position of the sequence channel in the ADC1_SQR3 register
 */
#define ADC_SEQUENCE_CHANNEL_POS 0 /**< Position of the channel in the sequence */

/**
 * @brief Define for the ADC channel used to read the battery
 */
#define ADC_CHANNEL_BATT 4 /**< ADC channel for reading battery voltage */

/**
 * @brief Define for the ADC channel used to read the temperature
 */
#define ADC_CHANNEL_TEMP 5 /**< ADC channel for reading temperature */

/**
 * @brief Define for enabling the ADC (ADC1)
 */
#define ADC_ENABLE ADC_CR2_ADON /**< Set the ADON bit to enable ADC1 */

/**
 * @brief Define for disabling the ADC (ADC1)
 */
#define ADC_DISABLE (~ADC_CR2_ADON) /**< Clear the ADON bit to disable ADC1 */

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
 * @brief Baud rate for UART communication.
 */
#define UART_BAUD_RATE 9600 /**< UART baud rate */

/**
 * @brief Number of data bits in UART communication.
 */
#define UART_DATA_BITS 8 /**< UART data bits */

/* -------------------------------- Function prototypes ------------------------------------------- */

/**
 * @brief Initializes the system clock to 72 MHz using an 8 MHz external crystal.
 */
void system_clock_setup(void);

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
void gpio_setup(void);

/**
 * @brief Configures and enables external interrupts (EXTI) for multiple GPIO pins.
 *
 * This function sets up the AFIO clock, enables the necessary NVIC interrupts,
 * and configures EXTI lines 0 to 3 for specific GPIO ports and trigger conditions.
 *
 * - EXTI0 (PA0) is linked to the `SWITCH_PORT` and configured for both edge triggers.
 * - EXTI1 (PA1) is linked to the `OVERRIDE_PORT` and configured for both edge triggers.
 * - EXTI2 (PA2) is linked to the `MOTION_SENSOR_PORT` and configured for both edge triggers.
 * - EXTI3 (PA3) is linked to the `FIRE_SENSOR_PORT` and configured for both edge triggers.
 */
void exti_setup(void);

/**
 * @brief Configures ADC1 for single-channel operation with specific sample times and DMA support.
 *
 * This function sets up the ADC1 peripheral for single-channel conversions, specifies sample times
 * for channels 4 and 5, and enables DMA for efficient data transfer.
 *
 * Steps performed:
 * - Enables the clock for ADC1.
 * - Disables ADC1 to allow configuration.
 * - Sets the regular sequence length to 1 (single channel).
 * - Configures the sample times for channels 4 and 5 to 1.5 cycles.
 * - Starts the regular conversion process.
 * - Enables DMA mode for the ADC.
 */
void configure_adc(void);

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
void configure_usart(void);

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
void config_pwm(void);
