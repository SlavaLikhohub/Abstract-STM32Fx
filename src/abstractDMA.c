#include "abstractDMA.h"
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <stdint.h>
#include <stddef.h>

static uint32_t _abst_conv_dma(enum abst_dma dma)
{
    switch (dma) {
        case ABST_DMA_1:
            return DMA1;
            break;
        case ABST_DMA_2:
            return DMA2;
            break;
        default:
            return NULL;
    }
}

static uint32_t _abst_conv_dma_rcc(enum abst_dma dma)
{
    switch (dma) {
        case ABST_DMA_1:
            return RCC_DMA1;
            break;
        case ABST_DMA_2:
            return RCC_DMA2;
            break;
        default:
            return NULL;
    }
}

/**
 * Initialize a DMA to transfer data from peripheral to memory without FIFO buffer.
 *
 * :param dma: DMA number :c:type:`abst_dma`.
 * :param stream: Stream number.
 * :param channel: Channel number.
 * :param src_addr: Address of peripheral register address.
 * :param dest_addr: Address in memory where to write data.
 * :param prior: Priority of DMA requests.
 *     * 0 - LOW
 *     * 1 - MEDIUM
 *     * 2 - HIGH
 *     * 3 - VERY HIGH
 * :param items_num: Number of data words to transfer (1-65535)
 *      If this parameter is not 1 destenation pointer incrementation is turned on.
 * :param periph_size: ize of data to be transmitted :c:type:`abst_periph_size`.
 * :return: Error code according to :c:type:`abst_errors`
 */
enum abst_errors abst_init_dma_p2m_direct(enum abst_dma abst_dma, 
                                          uint8_t stream, 
                                          uint8_t channel, 
                                          uint32_t *src_addr, 
                                          uint32_t *dest_addr,
                                          uint8_t prior,
                                          uint32_t items_num,
                                          enum abst_periph_size periph_size)
{
    rcc_periph_clock_enable(_abst_conv_dma_rcc(abst_dma));

    uint32_t dma = _abst_conv_dma(abst_dma);
    if (dma == NULL)
        return ABST_WRONG_PARAMS;
    
    dma_disable_stream(dma, stream);

    uint32_t i = 0;
    // Wait untill stream will end up the transmission
    while ((DMA_SCR(dma, stream) & DMA_SxCR_EN)) {
        i++;
        if (i > 5e6)
            return ABST_UNKNOWN_ERROR;
    }

    dma_set_peripheral_address(dma, stream, src_addr);

    dma_set_memory_address(dma, stream, dest_addr);

    uint32_t opencm_channel = channel << DMA_SxCR_CHSEL_SHIFT;
    dma_channel_select(dma, stream, opencm_channel);

    dma_set_number_of_data(dma, stream, items_num);

    dma_set_priority(dma, stream, prior << 16);

    dma_enable_direct_mode(dma, stream); // Disable FIFO

    dma_set_transfer_mode(dma, stream, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

    if (items_num > 1)
        dma_enable_memory_increment_mode(dma, stream);
    else 
        dma_disable_memory_increment_mode(dma, stream);
    
    dma_set_peripheral_burst(dma, stream, DMA_SxCR_PBURST_SINGLE);

    dma_set_peripheral_size(dma, stream, periph_size << 11);

    dma_enable_circular_mode(dma, stream);

    dma_disable_double_buffer_mode(dma, stream);

    dma_enable_stream(dma, stream);

    return ABST_OK;
}