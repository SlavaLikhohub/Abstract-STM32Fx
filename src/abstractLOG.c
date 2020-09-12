#include "abstractLOG.h"

#include "abstractUSART.h"
#include <stdio.h>
#include <stdarg.h>

struct abst_usart LOG_USART = 
{
    .num = 3,
    .word_len = ABST_USART_WL_8_BITS,
    .num_stop_bits = ABST_USART_STOPBITS_1,
    .baud_rate = 9600,
};

struct abst_pin TX = {
    .port = ABST_GPIOB,
    .num = 10,
    .mode = ABST_MODE_AF,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin RX = {
    .port = ABST_GPIOB,
    .num = 11,
    .mode = ABST_MODE_AF,
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
 */
void abst_usart_log_init(void)
{
    abst_gpio_init(&TX);
    abst_gpio_init(&RX);
    abst_usart_init(&LOG_USART, 100);
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

    uint8_t N = 100;
    char buff[N];

    va_start(arg, format);
    vsnprintf(buff, N, format, arg);
    va_end(arg);

    abst_usart_send_text(&LOG_USART, buff);
}