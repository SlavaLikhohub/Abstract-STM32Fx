#include "abstractSTM32.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <stdint.h>
#define _REG_BIT(base, bit)		(((base) << 5) + (bit))

static inline uint32_t ab_opencm_port_conv(const struct pin pin)
{
    return PERIPH_BASE_AHB1 + 0x0400 * pin.port;
}

static inline uint32_t ab_opencm_rcc_conv(const struct pin pin)
{
    return _REG_BIT(0x30, pin.port);
}

/**
 * Initialize the pin with the setting that specified in struct pin.
 * 
 * :param pin: pin struct with filled parameters.
 */
void abst_gpio_init(const struct pin pin)
{
    uint32_t opencm_port = ab_opencm_port_conv(pin);

    rcc_periph_clock_enable(ab_opencm_rcc_conv(pin));
    gpio_mode_setup(opencm_port, pin.dir/* | pin.mode*/, pin.pull_up_down, 1 << pin.num);
    //gpio_set_output_options(opencm_port, pin.otype, pin.speed, 1 << pin.num);
    //abst_digital_write(pin, 0);
}

/**
 * Set value at the output of the pin.
 * 
 * :param pin: The pin struct with filled parameters.
 * :param value: The value to set. If is_inverse flag in struc pin is true value will be inversed.
 */
void abst_digital_write(const struct pin pin, bool value)
{
    uint32_t opencm_port = ab_opencm_port_conv(pin);
    value ^= pin.is_inverse;
    if (value)
        gpio_set(opencm_port, 1 << pin.num);
    else
        gpio_clear(opencm_port, 1 << pin.num);
}

/**
 * Read value from the pin via the input driver.
 * :param pin: The pin struct with filled parameters.
 * :return: Read value.
 */
bool abst_digital_read(const struct pin pin)
{
    return gpio_get(ab_opencm_port_conv(pin), 1 << pin.num) ^ pin.is_inverse;
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
