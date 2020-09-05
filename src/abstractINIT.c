#include "abstractINIT.h"
#include "abst_libopencm3.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>

/**
 * Helper function for initializing SysTick.
 * Called from :c:func:`abst_init`
 *
 * :param systick_fr: SysTick desired frequency.
 * :param anb: The current AHB frequency in Hz.
 */
void _abst_init_systick(uint32_t systick_fr, uint32_t anb)
{
    systick_counter_disable();
    
    systick_set_frequency(systick_fr, anb);

    systick_clear();
    // nvic_set_priority(NVIC_SYSTICK_IRQ, 0);
    nvic_enable_irq(NVIC_SYSTICK_IRQ);
    systick_interrupt_enable();
    systick_counter_enable();
}

/**
 * Helper function for initializing TIM1 for PWM.
 * Called from :c:func:`abst_init`
 *
 * :param anb: The current AHB frequency in Hz.
 * :param hard_pwm_freq: Hard pulse wide modulation desired frequency (using TIM1). 
 *      Set **NULL** to disable hard PWM
 */
void _abst_init_hard_pwm_tim1(uint32_t anb, uint32_t hard_pwm_freq)
{
    if (!hard_pwm_freq) // Quick exit
        return;
    
    rcc_periph_clock_enable(RCC_TIM1);

    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM1, anb / hard_pwm_freq / 256);

    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);

    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC2);
    timer_enable_oc_output(TIM1, TIM_OC3);
    timer_enable_oc_output(TIM1, TIM_OC4);

    timer_enable_break_main_output(TIM1);

    timer_set_period(TIM1, 254);
    
    timer_enable_counter(TIM1);
}

/**
 * Helper function for converting parameters from :c:type:`abst_pin` or :c:type:`abst_pin_group` to libopencm3 analog
 *
 * :param mode: GPIO Pin mode :c:type:`abst_pin_mode`
 * :param speed: GPIO Output Pin Speed :c:type:`abst_pin_speed`
 * :return: Mode descriptor from libopencm3.
 */
static uint8_t _abst_conv_mode(uint8_t mode, uint8_t speed)
{
#ifdef STM32F1
    uint8_t f1_mode = GPIO_MODE_INPUT; // Default
    
    if (mode == ABST_MODE_INPUT || mode == ABST_MODE_ANALOG)
        f1_mode = GPIO_MODE_INPUT;
    else if (speed == ABST_OSPEED_2MHZ)
        f1_mode = GPIO_MODE_OUTPUT_2_MHZ;
    else if (speed >= ABST_OSPEED_10MHZ)
        f1_mode = GPIO_MODE_OUTPUT_10_MHZ;
    else 
        f1_mode = GPIO_MODE_OUTPUT_50_MHZ;
    
    return f1_mode;
#endif // STM32F1

#ifdef STM32F4
    return mode;
#endif // STM32F4
}

/**
 * Helper function for converting parameters from :c:type:`abst_pin` or :c:type:`abst_pin_group` to libopencm3 analog
 *
 * :param mode: GPIO Pin mode :c:type:`abst_pin_mode`
 * :param otype: GPIO Output Pins Driver Type :c:type:`abst_pin_otype`
 * :return: Config descriptor from libopencm3.
 */
static uint8_t _abst_conv_cnf(uint8_t mode, uint8_t otype)
{
#ifdef STM32F1
    uint8_t f1_cnf = GPIO_CNF_INPUT_FLOAT; // Default
     if (mode == ABST_MODE_ANALOG || 
            (mode == ABST_MODE_OUTPUT && otype == ABST_OTYPE_PP))
        f1_cnf = GPIO_CNF_INPUT_ANALOG; // = GPIO_CNF_OUTPUT_PUSHPULL
    
    else if ((mode == ABST_MODE_INPUT || mode == ABST_MODE_OUTPUT) 
            && otype == ABST_OTYPE_OD)
        f1_cnf = GPIO_CNF_INPUT_FLOAT; // = GPIO_CNF_OUTPUT_OPENDRAIN

    else if ((mode == ABST_MODE_INPUT || mode == ABST_MODE_AF)
            && otype == ABST_OTYPE_PP)
        f1_cnf = GPIO_CNF_INPUT_PULL_UPDOWN; // = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL
    
    else if (mode == ABST_MODE_AF && otype == ABST_OTYPE_OD)
        f1_cnf = GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN;

    return f1_cnf;
#endif //STM32F1

    return 0; // If not STM32F1
}

/**
 * Helper function for converting GPIO Output Pin Speed :c:type:`abst_pin_speed` to libopencm3 analog
 *
 * :param speed: GPIO Output Pin Speed :c:type:`abst_pin_speed`
 * :return: Speed descriptor from libopencm3.
 */
static uint8_t _abst_conv_speed(uint8_t speed)
{
#ifdef STM32F4
    uint32_t f4_speed;
    switch (speed) {
        case ABST_OSPEED_2MHZ:
            f4_speed = GPIO_OSPEED_2MHZ;
            break;
        case ABST_OSPEED_10MHZ:
            /* Fall throught */
        case ABST_OSPEED_25MHZ:
            f4_speed = GPIO_OSPEED_25MHZ;
            break;
        case ABST_OSPEED_50MHZ:
            f4_speed = GPIO_OSPEED_50MHZ;
            break;
        case ABST_OSPEED_100MHZ:
            f4_speed = GPIO_OSPEED_100MHZ;
            break;
        default:
            f4_speed = GPIO_OSPEED_100MHZ;
    }
    return f4_speed;
#endif // STM32F4

    return 0; // If not STM32F4
}

/**
 * Helper function for pin initializing. 
 * Is called from :c:func:`abst_gpio_init` and :c:func:`abst_group_gpio_init`.
 *
 * :param port: GPIO Port identifier :c:type:`abst_pin_port`
 * :param mode: GPIO Pin mode :c:type:`abst_pin_mode`
 * :param speed: GPIO Output Pin Speed :c:type:`abst_pin_speed`
 * :param otype: GPIO Output Pins Driver Type :c:type:`abst_pin_otype`
 * :param pull_up_down: GPIO Output Pin Pullup :c:type:`abst_pull_up_down`
 * :param af: GPIO Alternate function (0-15)
 * :param num: Pin identifiers. If multiple pins are to be set, use bitwise OR '|' to separate them. 
 */
void _abst_init_pins(   uint8_t port, 
                        uint8_t mode, 
                        uint8_t speed, 
                        uint8_t otype, 
                        uint8_t pull_up_down,
                        uint8_t af,
                        uint16_t num)
{
#ifdef STM32F4
    uint32_t opencm_port = _abst_opencm_port_conv(port);

    rcc_periph_clock_enable(_abst_opencm_rcc_gpio_conv(port));

    uint8_t f4_mode = mode; // Order is the same
    uint8_t f4_pull_up_down = pull_up_down; // Order is the same
    uint8_t f4_otype = otype; // Order is the same
    uint8_t f4_speed = _abst_conv_speed(speed);

    gpio_mode_setup(opencm_port,
                    f4_mode, 
                    f4_pull_up_down, 
                    num);
    
    if (mode == ABST_MODE_AF)
        gpio_set_af(opencm_port, af, num);
    
    gpio_set_output_options(opencm_port, 
                            f4_otype, 
                            f4_speed, 
                            num);
#endif
#ifdef STM32F1
    uint32_t opencm_port = _abst_opencm_port_conv(port);

    rcc_periph_clock_enable(_abst_opencm_rcc_gpio_conv(port));
    
    uint8_t f1_mode = _abst_conv_mode(mode, speed);
    uint8_t f1_cnf = _abst_conv_cnf(mode, otype);

    gpio_set_mode(opencm_port, f1_mode, f1_cnf, num);
#endif
}