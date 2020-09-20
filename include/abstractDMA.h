#ifndef _ABSTRACT_DMA_H_
#define _ABSTRACT_DMA_H_

#include "abstractSTM32.h"
#include <stdint.h>

/** DMA Stream Peripheral Word Width */
enum abst_periph_size
{
    /** 8 bits */
    ABST_DMA_SxCR_PSIZE_8BIT = 0,
    /** 16 bits */
    ABST_DMA_SxCR_PSIZE_16BIT,
    /** 32 bits */
    ABST_DMA_SxCR_PSIZE_32BIT
};

/** DMA number */
enum abst_dma 
{
    /** DMA1 */
    ABST_DMA_1 = 0,
    /** DMA2 */
    ABST_DMA_2
};

enum abst_errors abst_init_dma_p2m_direct(enum abst_dma abst_dma, 
                                          uint8_t stream, 
                                          uint8_t channel, 
                                          uint32_t *src_addr, 
                                          uint32_t *dest_addr,
                                          uint8_t prior,
                                          uint32_t items_num,
                                          enum abst_periph_size periph_size);

void abst_dma_start(enum abst_dma abst_dma, uint8_t stream, uint8_t channel);

#endif // _ABSTRACT_DMA_H_
