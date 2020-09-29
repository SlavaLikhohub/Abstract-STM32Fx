#include "abstractLOG.h"

#include "abstractUSART.h"
#include <stdio.h>
#include <stdarg.h>

uint32_t log_buff = 0;

struct abst_usart LOG_USART = 
{
    .num = 3,
    .word_len = ABST_USART_WL_8_BITS,
    .num_stop_bits = ABST_USART_STOPBITS_1,
    .baud_rate = 9600,
};

struct abst_pin TX = {
#ifdef STM32F1
    .port = ABST_GPIOB,
    .num = 10,
#endif
#ifdef STM32F4
    .port = ABST_GPIOD,
    .num = 8,
#endif
    .mode = ABST_MODE_AF,
    .af = 7, // For STM32F4
    .af_dir = ABST_AF_OUTPUT,  // For STM32F1
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin RX = {
#ifdef STM32F1
    .port = ABST_GPIOB,
    .num = 11,
#endif
#ifdef STM32F4
    .port = ABST_GPIOD,
    .num = 9,
#endif
    .mode = ABST_MODE_AF,
    .af = 7, // For STM32F4
    .af_dir = ABST_AF_INPUT, // For STM32F1
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};


void usart3_isr(void)
{
    abst_usart_interraption_handler(&LOG_USART);
}
/**
 * Init the loggint using USART3
 * 
 * :param baud_rate: Baud rate of USART3
 */
void abst_usart_log_init(uint32_t baud_rate, uint32_t buffer)
{
    log_buff = buffer;
    
    LOG_USART.baud_rate = baud_rate;
    abst_gpio_init(&TX);
    abst_gpio_init(&RX);
    abst_usart_init(&LOG_USART, 500);
}

/*
 * Send log message using USART3
 */
void abst_usart_log_send(const char *msg)
{
    abst_usart_send_text(&LOG_USART, msg);
}

/*
 * Send log message using USART3 with formatting
 */
void abst_usart_log_send_format(const char *format, ...)
{
    va_list arg;

    uint8_t N = log_buff;
    char buff[N];

    va_start(arg, format);
    vsnprintf(buff, N, format, arg);
    va_end(arg);

    abst_usart_send_text(&LOG_USART, buff);
}
