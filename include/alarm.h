#include <libopencm3/stm32/i2c.h> /**< For I2C peripheral */
#include "system_response.h" /**< For read_temperature() */

#define WRITE 0 /**< Write mode */
#define READ 1  /**< Read mode */

#define DISPLAY_UNIT_ADDRESS 0x3F /**< Display unit address */
#define DISPLAY_TENS_ADDRESS 0x3E /**< Display tens address */

/**
 * @brief Convert a value to the display format
 * @param value The value to convert
 * @return The value in the display format
 */
uint8_t convert_to_display(uint8_t value);

/**
 * @brief Obtain the temperature value
 * @return The unit value in the display format
 */
uint8_t obtain_unit_value(void);

/**
 * @brief Obtain the temperature value
 * @return The tens value in the display format
 */
uint8_t obtain_tens_value(void);

/**
 * @brief Display the temperature value
 * @param value The value to display
 * @param address The address of the device
 * @param i2c The I2C peripheral
 */
void display_temperature(uint8_t value, uint8_t address, uint32_t i2c);

/**
 * @brief Update the I2C value
 */
void update_i2c_value(void);