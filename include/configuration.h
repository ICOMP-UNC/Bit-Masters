/**
 * @file configuration.h
 * @brief Configuration header file
 * 
 */

#include <libopencm3/stm32/gpio.h> /**< Include the GPIO peripheral library */
#include <libopencm3/stm32/rcc.h> /**< Include the RCC peripheral library */
#include <libopencm3/stm32/exti.h> /**< Include the EXTI peripheral library */
#include <libopencm3/cm3/nvic.h> /**< Include the NVIC peripheral library */
#include <libopencm3/stm32/timer.h> /**< Include the timer peripheral library */

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

/**
 * @brief Configures the timer to generate a PWM signal
 * 
 * The timer is configured to generate a PWM signal with a period of 10 ms and a duty cycle of 0%.
 * 
 */
void config_pwm(void);


