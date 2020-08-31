/**
 * This file will cover all difference in function names in libopencm3 
 * across different microcontrollers
 */

 /*
  * Check list
  * 
  * + F... => done in specifed STM32F, - => different (TODO)
  * F... the same in specifed STM32F...
  *
  * abstractSTM32.c
  * 
  * systick_set_frequency       S  Systick common for all cortexM
  * systick_counter_enable      S
  * systick_counter_disable     S
  * systick_interrupt_enable    S
  * systick_clear               S
  * 
  * nvic_enable_irq             F1, 4
  * 
  * rcc_periph_clock_enable     F1, 4
  * 
  * gpio_mode_setup             - Too different to change separately
  * gpio_set_output_options     - Implementing by making different abst_init_gpio
  * 
  * gpio_set                    F1, 4
  * gpio_clear                  F1, 4
  * gpio_port_write             F1, 4
  * gpio_toggle                 F1, 4
  * gpio_get                    F1, 4
  * gpio_port_read              F1, 4
  *
  */

#ifndef _ABST_LIBOPENCM3_H_
#define _ABST_LIBOPENCM3_H_

#include <libopencm3/stm32/gpio.h>

#ifdef STM32F1
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/rcc.h>
#endif

#ifdef STM32F1
inline uint32_t _abst_opencm_port_conv(const uint8_t port)
{
    // Definition at line 76 of file stm32/f1/memorymap.h.
    return PERIPH_BASE_APB2 + 0x0400 * (port + 2); 
}


inline uint32_t _abst_opencm_rcc_conv(const uint8_t port)
{
    // Definition at line 552 of file f1/rcc.h.
    return _REG_BIT(0x18, port + 2);
}

#endif

#ifdef STM32F4
inline uint32_t _abst_opencm_port_conv(const uint8_t port)
{
    return PERIPH_BASE_AHB1 + 0x0400 * port;
}
inline uint32_t _abst_opencm_rcc_conv(const uint8_t port)
{
    return _REG_BIT(0x30, port);
}
#endif

#endif //_ABST_LIBOPENCM3_H_