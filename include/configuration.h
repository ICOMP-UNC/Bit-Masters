#include <libopencm3/stm32/gpio.h> /**< Include the GPIO peripheral library */
#include <libopencm3/stm32/rcc.h> /**< Include the RCC peripheral library */
#include <libopencm3/stm32/exti.h> /**< Include the EXTI peripheral library */
#include <libopencm3/cm3/nvic.h> /**< Include the NVIC peripheral library */

#define ALARM_PORT GPIOA /**< Alarm port corresponds to port A */
#define ALARM_PIN GPIO5 /**< Define the alarm pin as PA5 */

#define MOTOR_PORT GPIOA /**< Motor port corresponds to port A */
#define MOTOR_PIN GPIO6 /**< Define the motor pin as PA6 */

#define MANUAL_SWITCH_PORT GPIOA /**< Manual switch port corresponds to port A */
#define MANUAL_SWITCH_PIN GPIO7 /**< Define the manual switch pin as PA7 */

#define OVERRIDE_SWITCH_PORT GPIOA /**< Override switch port corresponds to port A */
#define OVERRIDE_SWITCH_PIN GPIO8 /**< Define the override switch pin as PA8 */

/**
 * @brief Configures the GPIO pins for the alarm, motor, manual switch, and override switch
 * 
 * The alarm pin is configured as an output, the motor pin is configured as an output in pin 6, port A,
 * the manual switch pin is configured as an input in pin 7 port A, and the override switch pin is configured
 * as an input in pin 8 port A.
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