#pragma once

#include <libopencm3/stm32/adc.h>
#include "configuration.h"

#define ADC_RESOLUTION      4095 /**< 12-bit ADC max value */
#define TEMP_SCALE_FACTOR   5    /**< Scale factor to convert voltage difference to temperature */
#define PERCENTAGE_MAX      100  /**< Define the maximum percentage value (100%) */
#define PWM_CHANNEL_2       2    /**< PWM channel 2 */

/**
 * @brief Processes the raw ADC value to calculate temperature.
 *
 * This function converts a raw ADC value from a temperature sensor into a 
 * temperature in degrees Celsius. It assumes a 12-bit ADC with a reference 
 * voltage of 3.3V, where the sensor output scales accordingly. 
 * A typical temperature sensor such as the LM35 is assumed, which outputs 
 * a linear voltage proportional to temperature.
 *
 * @param value The raw ADC value, ranging from 0 to 4095 (for a 12-bit ADC).
 * @return uint16_t The calculated temperature in degrees Celsius.
 *
 * @note The calculation assumes:
 *       - The ADC resolution is 12 bits (0-4095).
 *       - The reference voltage is 3.3V.
 *       - An offset and scaling factor as per the sensor characteristics.
 */
uint16_t process_temperature(uint16_t value);

/**
 * @brief Reads the temperature sensor value from the ADC.
 * 
 * This function configures the ADC to read from the temperature sensor channel,
 * starts the conversion, waits for it to finish, and then returns the conversion result.
 *
 * @return uint16_t The ADC conversion result for the temperature sensor.
 * 
 * @note This function assumes that the ADC is properly initialized and configured
 *       to read the temperature sensor channel. The result returned is the raw
 *       ADC value, which needs further conversion to obtain the actual temperature.
 */
uint16_t read_temperature(void);

/**
 * @brief Set the PWM duty cycle for a specified output channel.
 *
 * This function calculates the PWM compare value based on the given duty cycle 
 * percentage (0-100%) and sets the duty cycle for the specified output channel 
 * of the timer (TIM1). It supports output channels 2 and 3 (OC2 and OC3) only. 
 * If the channel number is not 2 or 3, the function will default to channel 3.
 *
 * @param duty_cycle The duty cycle percentage for the PWM signal. It should be
 *                   a value between 0 and 100, where 0% is off and 100% is full speed.
 * @param output_channel The PWM output channel to set the duty cycle for. 
 *                       It should be either 2 or 3 (OC2 or OC3).
 *                       - If `output_channel == 2`, the duty cycle is set for OC2.
 *                       - If `output_channel == 3`, the duty cycle is set for OC3.
 *                       If any other value is provided, the duty cycle will be set for OC3 by default.
 *
 * @note The output_channel must be either 2 or 3. If an invalid channel is passed,
 *       the function will set the duty cycle for channel 3 (OC3).
 */
void set_pwm_duty_cycle(uint8_t duty_cycle, uint8_t output_channel);
