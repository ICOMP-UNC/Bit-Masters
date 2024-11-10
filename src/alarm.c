#include "alarm.h"

uint8_t convert_to_display(uint8_t value) {
    switch (value) {
        case 0: return 0x3F; /* Display "0" */
        case 1: return 0x06; /* Display "1" */
        case 2: return 0x5B; /* Display "2" */
        case 3: return 0x4F; /* Display "3" */
        case 4: return 0x66; /* Display "4" */
        case 5: return 0x6D; /* Display "5" */
        case 6: return 0x7D; /* Display "6" */
        case 7: return 0x07; /* Display "7" */
        case 8: return 0x7F; /* Display "8" */
        case 9: return 0x6F; /* Display "9" */
        default: return 0x3F; /* Display "0" */
    }
}

uint8_t obtain_unit_value(void) {
    uint16_t temperature = read_temperature(); /* Read the temperature */

    if (temperature > 100) {
        return 0x31 /* Display "r" for error */;
    }

    uint8_t unit = temperature % 10; /* Obtain the unit value */
    return convert_to_display(unit); /* Convert the value to the display format */
}

uint8_t obtain_tens_value(void) {
    uint16_t temperature = read_temperature();

    if (temperature > 100) { /* Check if the temperature is greater than 100 */
        return 0x79; /* Display "E" for error */
    }

    uint8_t tens = temperature / 10; /* Obtain the tens value */
    return convert_to_display(tens); /* Convert the value to the display format */
}

void display_temperature(uint8_t value, uint8_t address, uint32_t i2c) {

    i2c_peripheral_enable(i2c); /* Enable the I2C peripheral */
    
    i2c_send_start(i2c); /* Send the start condition */
    i2c_send_7bit_address(i2c, address, WRITE); /* Send the address of the device */
    while (!(i2c_get_data(i2c) & (1 << 1)));        /* Wait for the address to be sent */

    i2c_send_data(i2c, value);                       /* Send the data */

}

void update_i2c_value(void) {

    uint8_t tens = obtain_tens_value(); /* Obtain the tens value */
    uint8_t unit = obtain_unit_value(); /* Obtain the unit value */

    while (!(i2c_get_data(I2C1_BASE) & (1 << 1)));        /* Wait for the data to be sent */
    i2c_send_stop(I2C1_BASE);                             /* Send the stop condition */

    i2c_peripheral_disable(I2C1_BASE);        /* Disable the I2C peripheral */

    display_temperature(tens, DISPLAY_UNIT_ADDRESS, I2C1_BASE);

    while (!(i2c_get_data(I2C2_BASE) & (1 << 1)));        /* Wait for the data to be sent */
    i2c_send_stop(I2C2_BASE);                             /* Send the stop condition */

    i2c_peripheral_disable(I2C2_BASE);        /* Disable the I2C peripheral */

    display_temperature(unit, DISPLAY_TENS_ADDRESS, I2C2_BASE); /* Send the data */

}
