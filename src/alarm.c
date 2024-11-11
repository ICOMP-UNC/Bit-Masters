#include "alarm.h"

uint16_t get_battery_value(void) {
    /* Configure the ADC */
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL_BATTERY_LEVEL); /* Select only one channel */

    /* Start the conversion */
    adc_start_conversion_regular(ADC1);

    /* Wait for the conversion to complete */
    while (!(ADC1_SR & ADC_SR_EOC));

    /* Read the conversion result */
    return adc_read_regular(ADC1) * CONVERT_VALUE;
}

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

    if (temperature > 99) {
        return 0x31 /* Display "r" for error */;
    }

    uint8_t unit = temperature % 10; /* Obtain the unit value */
    return convert_to_display(unit); /* Convert the value to the display format */
}

uint8_t obtain_tens_value(void) {
    uint16_t temperature = read_temperature();

    if (temperature > 99) { /* Check if the temperature is greater than 99 */
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

void update_i2c_value(uint32_t i2c, uint8_t data, uint8_t address) {

    while (!(i2c_get_data(ic2) & (1 << 1)));        /* Wait for the data to be sent */
    i2c_send_stop(ic2);                             /* Send the stop condition */

    i2c_peripheral_disable(ic2);        /* Disable the I2C peripheral */

    display_temperature(data, address, i2c); /* Display the temperature value */

}

void show_display(void){
    
    uint8_t unit = obtain_unit_value(); /* Obtain the unit value */
    uint8_t tens = obtain_tens_value(); /* Obtain the tens value */

    update_i2c_value(I2C1_BASE, unit, DISPLAY_UNIT_ADDRESS); /* Update the I2C value */
    update_i2c_value(I2C2_BASE, tens, DISPLAY_TENS_ADDRESS); /* Update the I2C value */
}

void activate_alarm(void) {
    if(gpio_get(ALARM_PORT, ALARM_PIN)) { /**< Check if the alarm is already active */
        return; /**< Return if the alarm is already active */
    }
    gpio_set(ALARM_PORT, ALARM_PIN); /**< Set the alarm pin high */
}

void deactivate_alarm(void) {
    if(!gpio_get(ALARM_PORT, ALARM_PIN)) { /**< Check if the alarm is already inactive */
        return; /**< Return if the alarm is already inactive */
    }
    gpio_clear(ALARM_PORT, ALARM_PIN); /**< Set the alarm pin low */
}
