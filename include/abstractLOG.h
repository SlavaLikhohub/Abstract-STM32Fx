/**
 * Module for logging using USART3. 
 *
 * NOTE: A lot of hard code. Tested only with STM32F103.
 * To have more frexebility use AbstractUSART module.
 */

#ifndef _ABSTRACT_LOG_H_
#define _ABSTRACT_LOG_H_


#if !((defined(LOG) && LOG) || defined(HAWKMOTH))

#define abst_log_init(baud_rate, buffer)
#define abst_log(msg)
#define abst_logf(msg, ...)

#else // !(defined(LOG) && LOG)

/**
 * Init the loggint using USART3. 
 * This function has effect only if **LOG** variable is declared and not equal to 0
 * 
 * :param baud_rate: Baud rate of USART connection
 */
#define abst_log_init(baud_rate, buffer) abst_usart_log_init(baud_rate, buffer)

/**
 * Send log message using USART3
 * This function has effect only if **LOG** variable is declared and not equal to 0
 */
#define abst_log(msg) abst_usart_log_send(msg)

/**
 * Send log message using USART3 with formatting
 * This function has effect only if **LOG** variable is declared and not equal to 0
 */
#define abst_logf(msg, ...) abst_usart_log_send_format(msg,##__VA_ARGS__)

#endif // !(defined(LOG) && LOG)

#include <stdint.h>

void abst_usart_log_init(uint32_t baud_rate, uint32_t buffer);

void abst_usart_log_send(const char *msg);

void abst_usart_log_send_format(const char *format, ...);

#endif // _ABSTRACT_LOG_H_
