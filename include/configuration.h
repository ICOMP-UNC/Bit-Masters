/**
 * @file configuration.h
 * @brief Configuration header file
 * 
 */
#pragma once

#include <libopencm3/stm32/gpio.h> /**< Include the GPIO peripheral library */
#include <libopencm3/stm32/rcc.h> /**< Include the RCC peripheral library */
#include <libopencm3/stm32/exti.h> /**< Include the EXTI peripheral library */
#include <libopencm3/cm3/nvic.h> /**< Include the NVIC peripheral library */
#include <libopencm3/stm32/timer.h> /**< Include the timer peripheral library */
#include <libopencm3/stm32/i2c.h> /**< Include the I2C library */

#define ALARM_PORT GPIOA /**< Alarm port corresponds to port A */
#define ALARM_PIN GPIO5 /**< Define the alarm pin as PA5 */

#define MOTOR_PORT GPIOA /**< Motor port corresponds to port A */
#define MOTOR_PIN GPIO6 /**< Define the motor pin as PA6 */

#define MANUAL_SWITCH_PORT GPIOA /**< Manual switch port corresponds to port A */
#define MANUAL_SWITCH_PIN GPIO7 /**< Define the manual switch pin as PA7 */

#define OVERRIDE_SWITCH_PORT GPIOA /**< Override switch port corresponds to port A */
#define OVERRIDE_SWITCH_PIN GPIO8 /**< Define the override switch pin as PA8 */

#define LED_PORT GPIOA /**< LED port corresponds to port A */
#define LED_PIN GPIO10 /**< Define the LED pin as PA10 */

#define FAN_PORT GPIOA /**< Fan port corresponds to port A */
#define FAN_PIN GPIO9 /**< Define the fan pin as PA9 */

#define TEMP_SENSOR_PORT GPIOA /**< Temperature sensor port corresponds to port A */
#define TEMP_SENSOR_PIN GPIO0 /**< Define the temperature sensor pin as PA0 */

#define BATTERY_LEVEL_PORT GPIOA /**< Battery level port corresponds to port A */
#define BATTERY_LEVEL_PIN GPIO1 /**< Define the battery level pin as PA1 */

#define MOTION_SENSOR_PORT GPIOA /**< Motion sensor port corresponds to port A */
#define MOTION_SENSOR_PIN GPIO2 /**< Define the motion sensor pin as PA2 */

#define INFRARED_SENSOR_PORT GPIOA /**< Infrared sensor port corresponds to port A */
#define INFRARED_SENSOR_PIN GPIO3 /**< Define the infrared sensor pin as PA3 */
  
/**
 * @brief I2C1 rise time in standard mode (100 kHz).
 * 
 * This define sets the maximum rise time for the I2C1 peripheral in 
 * standard mode (100 kHz) according to the I2C specifications. Adjust 
 * if the system clock changes.
 */
#define I2C1_TRISE_100KHZ 36

/**
 * @brief I2C1 CCR value for 100 kHz.
 * 
 * This define configures the Clock Control Register (CCR) of the 
 * I2C1 peripheral for standard mode at a frequency of 100 kHz. 
 * Adjust if a different speed is required.
 */
#define I2C1_CCR_100KHZ 180

/**
 * @brief I2C CR2 frequency setting for 36 MHz system clock.
 * 
 * This define sets the frequency for the I2C peripheral in the CR2 register 
 * when the system clock is 36 MHz. The value corresponds to the system clock 
 * frequency used to set the I2C communication speed. 
 * Adjust the value according to the actual system clock frequency.
 */
#define I2C_CR2_FREQ_36MHZ  36

#define PRESCALER_VALUE 71999 /**< Define the prescaler value for the timer */
/* Calculate it as follows: (timer_clock / desired_frequency) - 1
 * 78MHz / (10000) - 1 */
#define TIMER_PERIOD 0xFFFF /**< Full period of the timer */

static uint32_t duty_cycle = 0; /**< Initialize the duty cycle to 0 */

/**
 * @brief Initializes the system clock to 72 MHz using an 8 MHz external crystal.
 */
void system_clock_setup(void);

/**
 * @brief Configures the GPIO pins for the alarm, motor, manual switch, override switch, LED, fan, and sensors
 * 
 * The alarm pin is configured as an output, the motor pin is configured as an output in pin 6, port A,
 * the manual switch pin is configured as an input in pin 7 port A, the override switch pin is configured
 * as an input in pin 8 port A, the LED pin is configured as an output in pin 10 port A, the fan pin is configured
 * as an output in pin 9 port A, the temperature sensor pin is configured as an input in pin 0 port A, the battery
 * level pin is configured as an input in pin 1 port A, the motion sensor pin is configured as an input in pin 2 port A,
 * and the infrared sensor pin is configured as an input in pin 3 port A.
 * 
 */
void configure_gpio(void);

/**
 * @brief Configures the EXTI interrupt for the override switch
 * 
 * The AFIO clock is enabled, the EXTI9_5 interrupt is enabled in the NVIC, EXTI8 is configured as the source
 * for the override switch pin, the interrupt is triggered on the falling edge, and the EXTI8 interrupt is enabled.
 * 
 * Note: The EXTI9_5 manages EXTI5 to EXTI9 interrupts.
 */
void exti_setup(void);

 /** @brief Configures the I2C1 peripheral on the STM32.
 *
 * This function sets up the I2C1 peripheral on the STM32 microcontroller to operate
 * in standard mode (100 kHz) with GPIO pins configured for open-drain alternate function.
 * It enables the clocks for GPIOB and I2C1, configures the necessary pins for I2C communication,
 * sets the I2C clock frequency, rise time, and CCR value for a 100 kHz bus speed.
 *
 * @note Assumes a system clock frequency of 36 MHz for proper timing configuration.
 *       Adjustments may be necessary for different clock speeds.
 *
 * GPIOB is used for I2C1_SCL and I2C1_SDA lines with open-drain configuration.
 * Before enabling the I2C peripheral, it is disabled to ensure a clean configuration.
 */
void config_i2c(void);

/**
 * @brief Configures the timer to generate a PWM signal
 * 
 * The timer is configured to generate a PWM signal with a period of 10 ms and a duty cycle of 0%.
 * 
 */
void config_pwm(void);
