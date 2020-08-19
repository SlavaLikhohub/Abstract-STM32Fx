#include "abstractSTM32.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <list.h>
#include <stdint.h>

#define _REG_BIT(base, bit)		(((base) << 5) + (bit))

static volatile uint32_t _time_ticks_;
static volatile bool _sleep_;

// Service variables
// List of pins that are currently used for software PWM
static list_t *soft_pwm_list;
static uint8_t _pwm_cnt_;
// Tick every 100 us
static const uint32_t systick_fr = 1e4;

inline static uint32_t ab_opencm_port_conv(const struct pin *pin_ptr)
{
    return PERIPH_BASE_AHB1 + 0x0400 * pin_ptr->port;
}

inline static uint32_t ab_opencm_rcc_conv(const struct pin *pin_ptr)
{
    return _REG_BIT(0x30, pin_ptr->port);
}

static void abst_soft_pwm_hander(void)
{
    list_node_t *node;
    list_iterator_t *it = list_iterator_new(soft_pwm_list, LIST_HEAD);
    
    // Reset all in first tick
    if (_pwm_cnt_ == 0) {
        while (node = list_iterator_next(it)) {
            const struct pin * pin_ptr = node->val;
            // If __pwm_value == 0 don't turn on the pin
            abst_digital_write(pin_ptr, pin_ptr->__pwm_value);
        }
    }
    else {
        while (node = list_iterator_next(it)) {
            const struct pin * pin_ptr = node->val;
            if (pin_ptr->__pwm_value == _pwm_cnt_)
                abst_digital_write(pin_ptr, 0);
        }
    }
    free(it);
    // Reset to 0 from 255 by overflowing
    _pwm_cnt_++;
}

/**
 * Initialize the library. 1) Start systick
 * :param ahb: The current AHB frequency in Hz.
 */
void abst_init(uint32_t anb)
{
    _time_ticks_ = 0;
    _pwm_cnt_ = 0;
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


void abst_sys_tick_handler(void)
{
    _time_ticks_++;
    // If soft_pwm_list is not empty
    if (soft_pwm_list->len)
        abst_soft_pwm_hander();
}

/**
 * SysTick interruption handler.
 * In case of redefining of this function make sure
 * to call :c:func:`abst_sys_tick_handler` to keep soft PWM and delay alive
 */
void sys_tick_handler(void)
{
    abst_sys_tick_handler();
}

/**
 * Initialize the pin with the setting that specified in struct pin.
 * 
 * :param pin: pin struct with filled parameters.
 */
void abst_gpio_init(const struct pin *pin_ptr)
{
    uint32_t opencm_port = ab_opencm_port_conv(pin_ptr);

    rcc_periph_clock_enable(ab_opencm_rcc_conv(pin_ptr));
    gpio_mode_setup(opencm_port, 
                    pin_ptr->dir | pin_ptr->mode, 
                    pin_ptr->pull_up_down, 
                    1 << pin_ptr->num);
    
    gpio_set_output_options(opencm_port, 
                            pin_ptr->otype, 
                            pin_ptr->speed, 
                            1 << pin_ptr->num);
    abst_digital_write(pin_ptr, 0);
}

/**
 * Set value at the output of the pin.
 * 
 * :param pin: The pin struct with filled parameters.
 * :param value: The value to set. If is_inverse flag in struc pin is true value will be inversed.
 */
void abst_digital_write(const struct pin *pin_ptr, bool value)
{
    uint32_t opencm_port = ab_opencm_port_conv(pin_ptr);
    value ^= pin_ptr->is_inverse;
    if (value)
        gpio_set(opencm_port, 1 << pin_ptr->num);
    else
        gpio_clear(opencm_port, 1 << pin_ptr->num);
}

/**
 * Toggle value of a pin 
 *
 * :param pin: The pin struct with filled parameters.
 */
void abst_toggle(const struct pin *pin_ptr)
{
    gpio_toggle(ab_opencm_port_conv(pin_ptr), 1 << pin_ptr->num);
}

/**
 * Read value from the pin via the input driver.
 * :param pin: The pin struct with filled parameters.
 * :return: Read value.
 */
bool abst_digital_read(const struct pin *pin_ptr)
{
    return gpio_get(ab_opencm_port_conv(pin_ptr), 1 << pin_ptr->num) ^ pin_ptr->is_inverse;
}

/**
 * Set Pulse Wide Modulation at the pin.
 * :param pin: The pin struct with filled parameters.
 * :param value: the duty cycle: between 0 (always off) and 255 (always on).
 */
void abst_pwm_soft(struct pin *pin_ptr, uint8_t value)
{
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
    abst_digital_write(pin_ptr, 0);
    return true;
}

/**
 * Read analog value from the pin via the Analog to Digital Converter (!) TODO 
 * :param pin: The pin struct with filled parameters.
 * :return: Read value (12 bit)
 */
uint16_t abst_adc_read(struct pin *pin_ptr)
{
    return 0;
}

/**
 * Stop program for a given time.
 * :param miliseconds: Time to wait.
 */
void abst_delay_ms(uint64_t miliseconds)
{
    volatile uint32_t stp_time = abst_time_ms() + miliseconds;
    while (abst_time_ms() < stp_time)
        abst_sleep_wfi();
    abst_stop_sleep();
}

/**
 * Go to wait for interruption mode and set :c:data:`_sleep_` to true.
 * After interrupt by which microcontroller should be woken up permanentry call :c:func:`stop_sleep`.
 */
void abst_sleep_wfi(void)
{
    _sleep_ = true;
    __asm__ volatile("wfi");
}

/**
 * Wake up microcontroller permanently (set :c:data:`_sleep_` to false).
 */
void abst_stop_sleep(void)
{
    _sleep_ = false;
}
/**
 * Get time from Initialization in miliseconds. Timer overflow in 4 days 23 hours 18 minutes 
 * :return: Time in miliseconds
 */
uint32_t abst_time_ms(void)
{
    return _time_ticks_ * (1e3 / systick_fr);
}