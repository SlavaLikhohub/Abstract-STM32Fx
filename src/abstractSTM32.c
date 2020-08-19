#include "abstractSTM32.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <list.h>
#include <stdint.h>

#define _REG_BIT(base, bit)		(((base) << 5) + (bit))

volatile uint64_t _time_us_;
volatile bool _sleep_;

// Service variables
// List of pins that are currently used for software PWM
static list_t *soft_pwm_list;
// Tick every 100 us
static const uint32_t systick_fr = 1e4;

inline static uint32_t ab_opencm_port_conv(const struct pin pin)
{
    return PERIPH_BASE_AHB1 + 0x0400 * pin.port;
}

inline static uint32_t ab_opencm_rcc_conv(const struct pin pin)
{
    return _REG_BIT(0x30, pin.port);
}

/**
 * Initialize the library. 1) Start systick
 * :param ahb: The current AHB frequency in Hz.
 */
void abst_init(uint32_t anb)
{
    _time_us_ = 0;
    _sleep_ = false;
    soft_pwm_list = list_new();

    // SYSTICK for delays and PWMs
    systick_counter_disable();
    
    systick_set_frequency(systick_fr, anb);

    systick_clear();
    // nvic_set_priority(NVIC_SYSTICK_IRQ, 0);
    nvic_enable_irq(NVIC_SYSTICK_IRQ);
    systick_interrupt_enable();
    systick_counter_enable();
}


/**
 * SysTick interruption handler.
 * In case of redefining of this function make sure to call sys_tick_handler
 * to call abst_sys_tick_handler to keep PWM and delay alive
 */
void sys_tick_handler(void)
{
    abst_sys_tick_handler();
}

inline void abst_sys_tick_handler(void)
{
    _time_us_++;
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
    gpio_mode_setup(opencm_port, pin.dir | pin.mode, pin.pull_up_down, 1 << pin.num);
    gpio_set_output_options(opencm_port, pin.otype, pin.speed, 1 << pin.num);
    abst_digital_write(pin, 0);
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
 * Toggle value of a pin 
 *
 * :param pin: The pin struct with filled parameters.
 */
void abst_toggle(const struct pin pin)
{
    gpio_toggle(ab_opencm_port_conv(pin), 1 << pin.num);
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
 * Set Pulse Wide Modulation at the pin.
 * :param pin: The pin struct with filled parameters.
 * :param value: the duty cycle: between 0 (always off) and 255 (always on).
 */
void abst_pwm_soft(struct pin *pin_ptr, uint8_t value)
{
    // if (value == 255) {
    //     abst_digital_write(pin, 1);
    //     return;
    // }
    // else if (value == 0) {
    //     abst_digital_write(pin, 0);
    //     return;
    // }

    pin_ptr->__pwm_value = value;
    if (!list_find(soft_pwm_list, pin_ptr))
        list_lpush(soft_pwm_list, list_node_new(pin_ptr));
}

bool abst_stop_pwm_soft(struct pin *pin_ptr)
{
    list_node_t *list_pin = list_find(soft_pwm_list, pin_ptr);
    if (!list_pin)
        return false;
    
    list_remove(soft_pwm_list, list_pin);
    return true;
}

/**
 * Read analog value from the pin via the Analog to Digital Converter (!) TODO 
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
void delay_ms(uint64_t miliseconds)
{
    volatile uint64_t stp_time = _time_us_ + miliseconds;
    while (_time_us_ < stp_time)
        sleep_wfi();
    stop_sleep();
}

/**
 * Go to wait for interruption mode and set :c:data:`_sleep_` to true.
 * After interrupt by which microcontroller should be woken up permanentry call :c:func:`stop_sleep`.
 */
void sleep_wfi(void)
{
    _sleep_ = true;
    __asm__ volatile("wfi");
}

/**
 * Wake up microcontroller permanently (set :c:data:`_sleep_` to false).
 */
void stop_sleep(void)
{
    _sleep_ = false;
}