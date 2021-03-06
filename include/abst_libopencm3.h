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
#include <stdint.h>

#ifdef STM32F1

#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/rcc.h>

#endif // STM32F1

// COMMON DEFINITIONS
uint32_t _ABST_REG_BIT(uint32_t base, uint32_t bit);

//#define _ABST_REG_BIT(base, bit)		(((base) << 5) + (bit))

uint32_t _abst_opencm_port_conv(const uint8_t port);

uint32_t _abst_opencm_rcc_gpio_conv(const uint8_t port);


#endif //_ABST_LIBOPENCM3_H_