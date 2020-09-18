#include "abst_libopencm3.h"

uint32_t _ABST_REG_BIT(uint32_t base, uint32_t bit)
{
    return (((base) << 5) + (bit));
}

#ifdef STM32F1 // =======================================
inline uint32_t _abst_opencm_port_conv(const uint8_t port)
{
    // Definition at line 76 of file stm32/f1/memorymap.h.
    return PERIPH_BASE_APB2 + 0x0400 * (port + 2); 
}


inline uint32_t _abst_opencm_rcc_gpio_conv(const uint8_t port)
{
    // Definition at line 552 of file f1/rcc.h.
    return _ABST_REG_BIT(0x18, port + 2);
}

#endif // STM32F1 ============================================

#ifdef STM32F4 // ============================================

uint32_t _abst_opencm_port_conv(const uint8_t port)
{
    return PERIPH_BASE_AHB1 + 0x0400 * port;
}

uint32_t _abst_opencm_rcc_gpio_conv(const uint8_t port)
{
    return _ABST_REG_BIT(0x30, port);
}
#endif // STM32F4 ==============================================