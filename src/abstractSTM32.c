#include "abstractSTM32.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/**
 * Initialize the pin with the setting that specified in struct pin.
 * 
 * :param pin: pin struct with filled parameters.
 */
void abst_gpio_init(struct pin pin)
{
    rcc_periph_clock_enable(pin.port);
    gpio_mode_setup(pin.port, pin.dir | pin.mode, pin.pull_up_down, 1 << pin.num);
    gpio_set_output_options(pin.port, pin.otype, pin.speed, 1 << pin.num);
    gpio_clear(pin.port, 1 << pin.num);
}

/**
 * Set value at the output of the pin.
 * 
 * :param pin: The pin struct with filled parameters.
 * :param value: The value to set. If is_inverse flag in struc pin is true value will be inversed.
 */
void abst_digital_write(struct pin pin, bool value)
{
    if (value)
        gpio_set(pin.port, 1 << pin.num);
    else
        gpio_clear(pin.port, 1 << pin.num);
}

/**
 * Read value from the pin via the input driver.
 * :param pin: The pin struct with filled parameters.
 * :return: Read value.
 */
bool abst_digital_read(struct pin pin)
{
    return gpio_get(pin.port, 1 << pin.num);
}

/**
 * Set Pulse Wide Modulation at the pin. TO DO
 * :param pin: The pin struct with filled parameters.
 * :param value: the duty cycle: between 0 (always off) and 4095 (always on).
 */
void abst_pwm_write(struct pin pin, uint16_t value)
{
    if (value == 4095) {
        abst_digital_write(pin, 1);
        return;
    }
    else if (value == 0) {
        abst_digital_write(pin, 0);
        return;
    }
}

/**
 * Read analog value from the pin via the Analog to Digital Converter TO DO 
 * :param pin: The pin struct with filled parameters.
 * :return: Read value (12 bit)
 */
uint16_t abst_adc_read(struct pin pin)
{
    return 0;
}

/**
 * Stop program for a given time.
 * :param miliseconds: Time to wait.
 */
void delay(uint64_t miliseconds)
{
    delayMicroseconds(miliseconds * 1000);
}

/**
 * Stop program for a given time.
 * :param microseconds: Time to wait.
 */
void delayMicroseconds(uint64_t microseconds)
{
    rcc_periph_clock_enable(RCC_TIM1);
}