#ifndef _ABSTRACT_USART_H_
#define _ABSTRACT_USART_H_

#import "abstractSTM32.h"
#import "fifo_buffer.h"

/** Word length */
enum abst_usart_world_len
{
    /** 8 bits */
    ABST_USART_WL_8_BITS = 0,
    /** 9 bits */
    ABST_USART_WL_9_BITS
};

/** Number of stop bits */
enum abst_usart_num_stop_bits
{
    /** 1 bit */
    ABST_USART_STOPBITS_1 = 0,
    /** 0.5 bit */
    ABST_USART_STOPBITS_0_5,
    /** 2 bits */
    ABST_USART_STOPBITS_2,
    /** 1.5 bit */
    ABST_USART_STOPBITS_1_5
};

/**
 * Struct for storing data about USART
 */
struct abst_usart
{
    /** Number of USARD  */
    uint8_t num : 4;

    /** Word length :c:type:`abst_usart_world_len`. */
    enum abst_usart_world_len word_len : 1;

    /** Number of stop bits :c:type:`abst_usart_num_stop_bits` */
    enum abst_usart_num_stop_bits num_stop_bits : 2;

    /** Baud rate in Hz */
    uint32_t baud_rate;

    /* FIFO buffer for RX */
    struct fifo_buffer *_RX_buff;

    /* FIFO bugger for TX */
    struct fifo_buffer *_TX_buff;

    /* Indicated if USART is sending data */
    bool _sending;

    /* Indicated if USART is reciving data */
    bool _reciving;
};

enum abst_errors abst_usart_init(struct abst_usart *usart, uint32_t buff_max_size);

enum abst_errors abst_usart_send_msg(struct abst_usart *usart, uint16_t data[], size_t N);

void abst_usart_interraption_handler(struct abst_usart *usart);

enum abst_errors abst_usart_send_text(struct abst_usart *usart, char *text);

uint16_t abst_usart_read_elements(struct abst_usart *usart, size_t N, enum fifo_errors *err);

uint16_t abst_usart_read_element(struct abst_usart *usart, enum fifo_errors *err);

#endif //_ABSTRACT_USART_H_