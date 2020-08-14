#include <boost/test/auto_unit_test.hpp>
#include <string>
#include <iostream>
extern "C" 
{
#include "abstractSTM32.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
}
/*
 * Test of abstractSTM32Fx library
 * To build test call **make** command in **test** directory 
 */
extern inline uint32_t ab_opencm_port_conv(const struct pin pin);
BOOST_AUTO_TEST_CASE(portConvetror)
{
    struct pin led = {
        .port = AB_GPIOD,
        .num = 12,
        .dir = GPIO_MODE_OUTPUT,
        .mode = GPIO_OTYPE_PP,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_OSPEED_2MHZ,
        .pull_up_down = GPIO_PUPD_NONE,
        .is_inverse = false
    };
    BOOST_CHECK_EQUAL(ab_opencm_port_conv(led), GPIOD);
}

extern inline uint32_t ab_opencm_rcc_conv(const struct pin pin);
BOOST_AUTO_TEST_CASE(rccConvetror)
{
    struct pin led = {
        .port = AB_GPIOD,
        .num = 12,
        .dir = GPIO_MODE_OUTPUT,
        .mode = GPIO_OTYPE_PP,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_OSPEED_2MHZ,
        .pull_up_down = GPIO_PUPD_NONE,
        .is_inverse = false
    };
    BOOST_CHECK_EQUAL(ab_opencm_rcc_conv(led), RCC_GPIOD);
}