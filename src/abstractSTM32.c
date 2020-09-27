#include "abstractSTM32.h"
#include "abst_libopencm3.h"
#include "abstractINIT.h"
#include "abstractLOCK.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <cvector.h>

#define CEILING_POS(X) ((X-(int)(X)) > 0 ? (int)(X+1) : (int)(X))

static volatile uint32_t _time_ticks_;
static volatile bool _sleep_;

// Service variables
// Array of pins that are currently used for software PWM
cvector_vector_type(struct abst_pin *)soft_pwm_array;
static uint8_t pwm_array_len;
static uint8_t _pwm_cnt_;
abst_lock pwm_lock;


static uint32_t frequency;
static uint32_t systick_fr;
/*
 * Helper function for controling the soft PWM. 
 */
static void abst_soft_pwm_hander(void)
{
    uint16_t N = cvector_size(soft_pwm_array);
    // Reset all in first tick
    if (_pwm_cnt_ == 0) {
        for (uint16_t i = 0; i < N; i++) {
            struct abst_pin *pin_ptr = soft_pwm_array[i];
            
            if (!abst_lock_try_set(&pwm_lock)) // Skip if locked
                continue;
            
            pin_ptr->__pwm_value_set = 0;
            abst_lock_clear(&pwm_lock);
            // If __pwm_value == 0 don't turn on the pin
            abst_digital_write(pin_ptr, pin_ptr->__pwm_value);
        }
    }
    else {
        for (uint16_t i = 0; i < N; i++) {
            struct abst_pin *pin_ptr = soft_pwm_array[i];
            
            if (pin_ptr->__pwm_value <= _pwm_cnt_ && !pin_ptr->__pwm_value_set) {
                abst_digital_write(pin_ptr, 0);
                
                if (!abst_lock_try_set(&pwm_lock)) // Skip if locked
                    continue;
                pin_ptr->__pwm_value_set = 1;
                abst_lock_clear(&pwm_lock);
            }
        }
    }
        
    _pwm_cnt_++;
    if (_pwm_cnt_ == 255)
        _pwm_cnt_ = 0;
}

/*
 * Compose bits in the group.
 * Example: pins_num = 0b01001011, value = 0b0100010, output = 0b1010
 */
static uint16_t compose_bits(uint16_t pins_num, uint16_t values)
{
    volatile uint16_t output = 0;
    uint8_t cnt = 0;
    for (int i = 0; i < 16; i++) {
        if ((pins_num >> i) & 1) {
            output |= ((values >> i) & 1) << cnt;
            cnt++;
        }
    }
    return output;
}

/*
 * Decompose bits in the group.
 * Example: pins_num = 0b01001011, value = 0b1010, output = 0b0100010
 */
static uint16_t decompose_bits(uint16_t pins_num, uint16_t values)
{
    uint16_t output = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < sizeof(pins_num); i++) {
        if ((pins_num >> i) & 1) {
            output |= ((values >> cnt) & 1) << i;
            cnt++;
        }
    }
    return output;
}

/**
 * Initialize the library. 1) Reset global variables 2) Start systick
 *
 * :param ahb: The current AHB frequency in Hz. 
 * :param soft_pwm_freq: Frequency of soft PWM.
 *          It should be greater than 1e4/255 and less than AHB / (1000 * 255)
 * If frequency changes recall this function with different values.
 */
void abst_init(uint32_t anb, uint32_t soft_pwm_freq)
{
    static bool _inited_ = false;
    frequency  = anb;
    systick_fr = soft_pwm_freq * 256;
    pwm_lock = 0;
    if (systick_fr < 1e4)
        systick_fr = 1e4;
    
    if (!_inited_) {
        _time_ticks_ = 0;
        _pwm_cnt_ = 0;
        _sleep_ = false;
        _inited_ = true;
    }

    // SYSTICK for delays and PWMs
    _abst_init_systick(systick_fr, anb);
}


void abst_sys_tick_handler(void)
{
    _time_ticks_++;
    // If soft_pwm_array is not empty
    if (cvector_size(soft_pwm_array))
        abst_soft_pwm_hander();
}

/**
 * SysTick interruption handler. (Weak definition)
 *
 * In case of redefining of this function make sure
 * to call :c:func:`abst_sys_tick_handler` to keep soft PWM and delay alive
 */
void __attribute__ ((weak)) sys_tick_handler(void)
{
    abst_sys_tick_handler();
}

/**
 * Initialize the pin with the setting that specified in struct pin.
 * 
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 * :return: Error code :c:type:`abst_errors`.
 */
enum abst_errors abst_gpio_init(const struct abst_pin *pin_ptr)
{
    _abst_init_pins(pin_ptr->port, 
                    pin_ptr->mode,
                    pin_ptr->speed,
                    pin_ptr->otype,
                    pin_ptr->pull_up_down,
                    pin_ptr->af,
                    pin_ptr->af_dir,
                    1 << pin_ptr->num);

    // Setup channel of ADC
    if (pin_ptr->mode == ABST_MODE_ANALOG && pin_ptr->adc_sample_time != NULL) {
        enum abst_errors res = _abst_init_adc_channel(pin_ptr);
        if (res != ABST_OK)
            return res; // Initialization failed
    }

    return ABST_OK;
}

/**
 * Initialize the pin group with the setting that specified in struct pin.
 * 
 * :param pin_gr_ptr: Pointer to :c:type:`abst_pin_group` with filled parameters.
 * :return: Error code :c:type:`abst_errors`.
 */
enum abst_errors abst_group_gpio_init(const struct abst_pin_group *pin_gr_ptr)
{
    _abst_init_pins(pin_gr_ptr->port, 
                    pin_gr_ptr->mode,
                    pin_gr_ptr->speed,
                    pin_gr_ptr->otype,
                    pin_gr_ptr->pull_up_down,
                    pin_gr_ptr->af,
                    pin_gr_ptr->af_dir,
                    pin_gr_ptr->num);

    return ABST_OK;
}

/**
 * Set value at the output of the pin.
 * 
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 * :param value: The value to set. If is_inverse flag in :c:type:`abst_pin` is true value will be inversed.
 */
void abst_digital_write(const struct abst_pin *pin_ptr, bool value)
{
    uint32_t opencm_port = _abst_opencm_port_conv(pin_ptr->port);
    value ^= pin_ptr->is_inverse;
    if (value)
        gpio_set(opencm_port, 1 << pin_ptr->num);
    else
        gpio_clear(opencm_port, 1 << pin_ptr->num);
}

/**
 * Set value at the output of the pin group.
 * 
 * :param pin_gr_ptr: Pointer to :c:type:`abst_pin_group` struct with filled parameters.
 * :param value: The value to set. If is_inverse flag in :c:type:`abst_pin_group` is true value will be inversed.
 */
void abst_group_digital_write(const struct abst_pin_group *pin_gr_ptr, uint16_t values)
{
    uint32_t opencm_port = _abst_opencm_port_conv(pin_gr_ptr->port);
    
    // Init values readed from GPIO_ODR register
    uint16_t out_values = GPIO_ODR(opencm_port);
    
    values ^= pin_gr_ptr->is_inverse;
    
    int cnt = 0;
    for (uint8_t i = 0; i < sizeof(values) * 8; i++) {
        if ((pin_gr_ptr->num >> i) & 1) {
            out_values &= ~(1 << i);
            out_values |= ((values >> cnt++) & 1) << i;
        }
    }

    gpio_port_write(opencm_port, out_values);
}

/**
 * Toggle value of a pin 
 *
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 */
void abst_toggle(const struct abst_pin *pin_ptr)
{
    gpio_toggle(_abst_opencm_port_conv(pin_ptr->port), 1 << pin_ptr->num);
}

/**
 * Read value from the pin via the input driver.
 *
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 * :return: Read value.
 */
bool abst_digital_read(const struct abst_pin *pin_ptr)
{
    return gpio_get(_abst_opencm_port_conv(pin_ptr->port), 1 << pin_ptr->num) ^ pin_ptr->is_inverse;
}

/**
 * Read value from pins in a group via the input driver.
 *
 * :param pin_gr_ptr: Pointer to :c:type:`abst_pin_group` with filled parameters.
 * :return: Read bitmap.
 */
uint16_t abst_group_digital_read(const struct abst_pin_group *pin_gr_ptr)
{
    uint32_t opencm_port = _abst_opencm_port_conv(pin_gr_ptr->port);
    uint16_t values = gpio_port_read(opencm_port);
    uint16_t out = compose_bits(pin_gr_ptr->num, values);
    return out^pin_gr_ptr->is_inverse;
}

/**
 * Set Pulse Wide Modulation (PWM) at the pin using systick. When PWM should be stopped call :c:func:`abst_stop_pwm_soft`
 *
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 * :param value: the duty cycle: between 0 (always off) and 255 (always on).
 */
void abst_pwm_soft(struct abst_pin *pin_ptr, uint8_t value)
{
    pin_ptr->__pwm_value = value;
    bool found = false;
    uint16_t N = cvector_size(soft_pwm_array);
    for (uint16_t i = 0; i < N; i++) {
        if (soft_pwm_array[i] == pin_ptr) {
            found = true;
            break;
        }
    }
    if (!found) {
        abst_lock_spin(&pwm_lock);
        cvector_push_back(soft_pwm_array, pin_ptr);
        abst_lock_clear(&pwm_lock);
    }
}

/**
 * Stop PWM on the pin.
 *
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 * :return: true if operation successful, false if pin was not found in list of pins with PWM
 */
bool abst_stop_pwm_soft(struct abst_pin *pin_ptr)
{
    bool found = false;
    uint16_t N = cvector_size(soft_pwm_array);
    uint16_t i;
    for (i = 0; i < N; i++) {
        if (soft_pwm_array[i] == pin_ptr) {
            found = true;
            break;
        }
    }
    if (!found)
        return false;
    
    for (uint16_t j = i; j < N; j++) {
        soft_pwm_array[j] = soft_pwm_array[j + 1];
    }
    abst_lock_spin(&pwm_lock);
    cvector_pop_back(soft_pwm_array);
    abst_lock_clear(&pwm_lock);
    
    abst_digital_write(pin_ptr, 0);
    return true;
}

/**
 * Set Pulse Wide Modulation (PWM) at the pin using Advanced Timer TIM1.
 * 
 * Freequency of PWM specifying by calling :c:func:`abst_init`
 *
 * Hard PWM has only 4 channels:
 * 
 * * STM32F1, STM32F4
 *      * CH1 : PA8     PE9
 *      * CH2 : PA9     PE11
 *      * CH3 : PA10    PE13
 *      * CH4 : PA11    PE14
 *
 * Calling this function for other pins will have no effect 
 *
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 * :param value: the duty cycle: between 0 (always off) and 255 (always on).
 *      If pin->is_inverse == true, 0 - always on and 255 - always off.
 */
void abst_pwm_hard(struct abst_pin *pin_ptr, uint8_t value)
{
    uint8_t channel = 0;
    if (pin_ptr->port == ABST_GPIOA) {
        switch (pin_ptr->num) {
        case 8:
            channel = TIM_OC1;
            break;
        case 9:
            channel = TIM_OC2;
            break;
        case 10:
            channel = TIM_OC3;
            break;
        case 11:
            channel = TIM_OC4;
            break;
        default:
            return; // Invalid pin
        }
    }
    else if (pin_ptr->port == ABST_GPIOE) {
    switch (pin_ptr->num) {
        case 9:
            channel = TIM_OC1;
            break;
        case 11:
            channel = TIM_OC2;
            break;
        case 13:
            channel = TIM_OC3;
            break;
        case 14:
            channel = TIM_OC4;
            break;
        default:
            return; // Invalid pin
        }
    }
    else {
        return; // Invalid port
    }
    if (pin_ptr->is_inverse)
        value = 255 - value;
    
    timer_set_oc_value(TIM1, channel, value);

    if (pin_ptr->mode != ABST_MODE_AF || pin_ptr->af != 1) {
        pin_ptr->mode = ABST_MODE_AF;
        pin_ptr->af = 1; // TIM1/TIM2 in STM32F4. No effect in STM32F1

        abst_gpio_init(pin_ptr);
    }
    return;
}

/**
 * Stop program for a given time.
 *
 * :param miliseconds: Time to wait.
 */
void abst_delay_ms(uint32_t miliseconds)
{
    volatile uint32_t stp_time = _time_ticks_ + (miliseconds * (uint64_t)systick_fr) / (uint64_t)1e3 + 1;
    while (_time_ticks_ < stp_time)
        abst_sleep_wfi();
    abst_stop_sleep();
}
/**
 * Stop program for a given time. NOT FULLY IMPLEMENTED YET
 *
 * :param microseconds: Time to wait.
 */
void abst_delay_us(uint32_t microseconds)
{
    //abst_delay_ms((microseconds % 1000) ? (1 + microseconds / 1000) : (microseconds / 1000));
    volatile uint32_t us = microseconds;
    while (us-- * (frequency / (uint32_t)1e6) > 0);
}

/**
 * Go to wait for interruption mode and set :c:data:`_sleep_` to true.
 * After interrupt by which microcontroller should be woken up permanentry call :c:func:`abst_stop_sleep`.
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
 * Get time from Initialization in miliseconds. Timer overflows in 4 days 23 hours 18 minutes 
 *
 * :return: Time in miliseconds
 */
uint32_t abst_time_ms(void)
{
    return ((uint64_t)_time_ticks_ * (uint64_t)1e3) / (uint64_t)systick_fr;
}
