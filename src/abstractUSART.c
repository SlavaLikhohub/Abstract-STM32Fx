#include "abstractUSART.h"
#include "libopencm3/stm32/usart.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/cm3/nvic.h"
#include <stdlib.h>
#include <string.h>

static uint32_t _abst_conv_usart(uint8_t abst_usart)
{
    switch (abst_usart) {
        case 1:
            return USART1;
            break;
        case 2:
            return USART2;
            break;
        case 3:
            return USART3;
            break;
        case 4:
            return UART4;
            break;
        case 5:
            return UART5;
            break;
        default:
            return NULL;
    }
}

static uint32_t _abst_conv_usart_word_len(enum abst_usart_world_len word_len)
{
    switch (word_len) {
        case ABST_USART_WL_8_BITS:
            return 8;
            break;
        case ABST_USART_WL_9_BITS:
            return 9;
            break;
        default:
            return NULL;
    }
}

static uint32_t _abst_conv_usart_stop_bits(enum abst_usart_num_stop_bits stp_bits)
{
    switch (stp_bits) {
        case ABST_USART_STOPBITS_1:
            return USART_STOPBITS_1;
            break;
        case ABST_USART_STOPBITS_0_5:
            return USART_STOPBITS_0_5;
            break;
        case ABST_USART_STOPBITS_2:
            return USART_STOPBITS_2;
            break;
        case ABST_USART_STOPBITS_1_5:
            return USART_STOPBITS_1_5;
            break;
    }
}

static uint32_t _abst_conv_usart_irq(uint8_t abst_usart)
{
    switch (abst_usart) {
        case 1:
            return NVIC_USART1_IRQ;
            break;
        case 2:
            return NVIC_USART2_IRQ;
            break;
        case 3:
            return NVIC_USART3_IRQ;
            break;
        case 4:
            return NVIC_UART4_IRQ;
            break;
        case 5:
            return NVIC_UART5_IRQ;
            break;
        default:
            return NULL;
    }
}

static uint32_t _abst_conv_usart_rcc(uint8_t abst_usart)
{
    switch (abst_usart) {
        case 1:
            return RCC_USART1;
            break;
        case 2:
            return RCC_USART2;
            break;
        case 3:
            return RCC_USART3;
            break;
        case 4:
            return RCC_UART4;
            break;
        case 5:
            return RCC_UART5;
            break;
        default:
            return NULL;
    }
}

/*
 * Try to start sending data
 * If sending already in proccess - quick exit
 */
void abst_usart_start_sending(struct abst_usart *usart)
{
    if (usart->_sending == 1) // Already sending
        return;

    usart->_sending = 1;

    uint32_t opencm_usart = _abst_conv_usart(usart->num);

    enum fifo_errors err = FIFO_OK;
    uint16_t data = fifo_read_element(usart->_TX_buff, &err);
    if (err != FIFO_OK) {
        usart->_sending = 0;
        return;
    }

    usart_send(opencm_usart, data);

    usart_enable_tx_interrupt(opencm_usart);
}

/**
 * Initialize the USART connection
 * 
 * :param usart: A pointer to the :c:type:`abst_usart` with filled parameters. 
 *
 * :return: Error code according to :c:type:`abst_errors`
 */
enum abst_errors abst_usart_init(struct abst_usart *usart, uint32_t buff_max_size)
{
    usart->_sending = 0;
    usart->_reciving = 0;

    uint32_t opencm_usart = _abst_conv_usart(usart->num);
    if (opencm_usart == NULL)
        return ABST_WRONG_PARAMS;

    uint32_t opencm_usart_rcc = _abst_conv_usart_rcc(usart->num);
    if (opencm_usart_rcc == NULL)
        return ABST_WRONG_PARAMS;

    uint32_t opencm_word_len = _abst_conv_usart_word_len(usart->word_len);
    if (opencm_word_len == NULL)
        return ABST_WRONG_PARAMS;

    uint32_t opencm_usart_irq = _abst_conv_usart_irq(usart->num);
    if (opencm_usart_irq == NULL)
        return ABST_WRONG_PARAMS;    

    uint32_t opncm_stp_bits = _abst_conv_usart_stop_bits(usart->num_stop_bits);

    rcc_periph_clock_enable(opencm_usart_rcc);

    usart_enable(opencm_usart);

    usart_set_databits(opencm_usart, opencm_word_len);

    usart_set_stopbits(opencm_usart, opncm_stp_bits);

    usart_disable_tx_dma(opencm_usart);
    usart_disable_rx_dma(opencm_usart);

    usart_set_baudrate(opencm_usart, usart->baud_rate);

    usart_enable_rx_interrupt(opencm_usart);
    
    usart_set_mode(opencm_usart, USART_MODE_TX_RX);

    // Init buffers
    struct fifo_buffer _RX_buff = fifo_init(buff_max_size, malloc, free);

    usart->_RX_buff = malloc(sizeof(_RX_buff));
    *(usart->_RX_buff) = _RX_buff;


    struct fifo_buffer _TX_buff = fifo_init(buff_max_size, malloc, free);

    usart->_TX_buff = malloc(sizeof(_TX_buff));
    *(usart->_TX_buff) = _TX_buff;
    
    nvic_enable_irq(opencm_usart_irq);

    return ABST_OK;
}

/**
 * Send an array of data throught USART
 *
 * :param usart: A pointer to the :c:type:`abst_usart` after initialization by :c:func:`abst_usart_init`.
 * :param data: An array of data.
 * :param N: Length of the array.
 * :return: Error code acording to :c:type:`abst_errors`.
 */
enum abst_errors abst_usart_send_msg(struct abst_usart *usart, uint16_t data[], size_t N)
{
    enum fifo_errors err = fifo_add_elements(usart->_TX_buff, data, N);
    if (err != FIFO_OK)
        return ABST_UNKNOWN_ERROR;

    abst_usart_start_sending(usart);
}

/**
 * Send text throught USART
 *
 * :param usart: A pointer to the :c:type:`abst_usart` after initialization by :c:func:`abst_usart_init`.
 * :param text: Null-terminated string to be transmitted.
 * :return: Error code acording to :c:type:`abst_errors`.
 */
enum abst_errors abst_usart_send_text(struct abst_usart *usart, char *text)
{
    size_t N = strlen(text);
    uint16_t data[N];
    for (size_t i = 0; i < N; i++) {
        data[i] = text[i];
    }

    abst_usart_send_msg(usart, data, N);
}

/**
 * Read **N** messages from RX buffer.
 *
 * :param usart: A pointer to the :c:type:`abst_usart` after initialization by :c:func:`abst_usart_init`.
 * :param N: Number of elements to read
 * :param err: Status variable.
 *      After execution it holds error code acording to :c:type:`fifo_errors`
 * :return: Array of elements that was read from the buffer or **NULL** if reading failed.
 *      Array located in a **HEAP** and should be freed after using.
 */
uint16_t abst_usart_read_elements(struct abst_usart *usart, size_t N, enum fifo_errors *err)
{
    return fifo_read_elements(usart->_RX_buff, N, err);
}

/**
 * Read **1** element from the buffer
 *
 * :param buff: Pointer to the buffer :c:type:`fifo_buffer`.
 * :param err: Status variable. 
 *      After execution it holds error code acording to :c:type:`fifo_errors`.
 * :return: Element that was read from the buffer or **NULL** if reading failed.
 */
uint16_t abst_usart_read_element(struct abst_usart *usart, enum fifo_errors *err)
{
    return fifo_read_element(usart->_RX_buff, err);
}


/**
 * Interrauption handler. Should be placed in **usart{1-3}_isr** (see libopencm3 documentation).
 *
 * :param usart: A pointer to the :c:type:`abst_usart` after initialization by :c:func:`abst_usart_init`.
 */
void abst_usart_interraption_handler(struct abst_usart *usart)
{
    uint32_t opencm_usart = _abst_conv_usart(usart->num);
    
    if (usart_get_flag(opencm_usart, USART_SR_RXNE)) { // Read the value
        uint16_t data = usart_recv(opencm_usart);

        fifo_add_elements(usart->_RX_buff, &data, 1);

        return;
    }

    else if (usart_get_flag(opencm_usart, USART_SR_TXE)) {
        enum fifo_errors err = FIFO_OK;
        uint16_t data = fifo_read_element(usart->_TX_buff, &err);

        if (err != FIFO_OK) { // Stop transmitting, buffer is empty
            usart_disable_tx_interrupt(opencm_usart);
            usart->_sending = 0;
            return;
        }
        else { // Transmit and quit
            usart_send(opencm_usart, data);
            return;
        }
    }
}