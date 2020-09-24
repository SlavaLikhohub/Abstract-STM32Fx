#include "abstractADC.h"
#include "abstractDMA.h"
#include "abstractSTM32.h"
#include "abst_libopencm3.h"
#include <stdint.h>
#include <stddef.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>

static inline uint32_t _abst_opencm_adc_conv(const uint8_t adc_n)
{
#ifdef STM32F1
    return PERIPH_BASE_APB2 + 0x2400 + 0x400 * (adc_n - 1);
#endif
#ifdef STM32F4
    // Definition at line 79 of file stm32/f4/memorymap.h.
    return PERIPH_BASE_APB2 + 0x2000 + 0x100 * (adc_n - 1);
#endif // STM32F4
}

static inline uint32_t _abst_opencm_rcc_adc_conv(const uint8_t adc_n)
{
#ifdef STM32F1
    return _ABST_REG_BIT(0x18, adc_n + 9 - 1);
#endif
#ifdef STM32F4
    return _ABST_REG_BIT(0x44, adc_n + 7);
#endif // STM32F4
}


static inline uint8_t _abst_conv_adc_samle_time(uint8_t sample_time)
{
    return sample_time; // The same order
}

static uint32_t _abst_conv_adc_resolution(uint8_t resolution)
{
#ifdef STM32F1
    return 0; // Not implemented in STM32F1
#endif // STM32F1

#ifdef STM32F4
    switch (resolution) {
        case ABST_ADC_RES_12BIT:
            return ADC_CR1_RES_12BIT;
            break;
        case ABST_ADC_RES_10BIT:
            return ADC_CR1_RES_10BIT;
            break;
        case ABST_ADC_RES_8BIT:
            return ADC_CR1_RES_8BIT;
            break;
        case ABST_ADC_RES_6BIT:
            return ADC_CR1_RES_6BIT;
            break;
        default:
            return 0; // Wrong argument
    }
#endif // STM32F4
}

/**
 * Helper function to converting prescaler div ratio to libopencm3 analog
 * 
 * :param prescale: Prescale: 2, 4, 6, 8.
 * :return: libopencm3 definition or default value (PRESCK 2) if prescale is wrong.
 */ 
static uint32_t _abst_conv_adc_prescale(uint8_t prescale)
{
#ifdef STM32F1
    switch (prescale) {
        case 2:
            return RCC_CFGR_ADCPRE_PCLK2_DIV2;
            break;
        case 4:
            return RCC_CFGR_ADCPRE_PCLK2_DIV4;
            break;
        case 6:
            return RCC_CFGR_ADCPRE_PCLK2_DIV6;
            break;
        case 8:
            return RCC_CFGR_ADCPRE_PCLK2_DIV8;
            break;
        default:
            return RCC_CFGR_ADCPRE_PCLK2_DIV2;
    }
#endif // STM32F1
#ifdef STM32F4
    switch (prescale) {
        case 2:
            return ADC_CCR_ADCPRE_BY2;
            break;
        case 4:
            return ADC_CCR_ADCPRE_BY4;
            break;
        case 6:
            return ADC_CCR_ADCPRE_BY6;
            break;
        case 8:
            return ADC_CCR_ADCPRE_BY8;
            break;
        default:
            return ADC_CCR_ADCPRE_BY2;
    }
#endif // STM32F4
}

/**
 * Helper function for initializing an ADC in single convesation mode with scan mode disabled
 * After initializing an ADC is turned off.
 * Called from :c:func:`abst_init`
 *
 * :param adc_n: Number of ADC (1-3). If specified another the function will return :c:member:`abst_errors.ABST_WRONG_PARAMS`
 * :param prescale: Prescale: 2, 4, 6, 8. If specified another the function will return :c:member:`abst_errors.ABST_WRONG_PARAMS`
 * :return: Error code according to :c:type:`abst_errors`.
 */
enum abst_errors abst_adc_init_single_conv(uint8_t adc_n, uint8_t prescale)
{
    if (adc_n < 1 || adc_n > 3)
        return ABST_WRONG_PARAMS;

    uint8_t opencm_presc = _abst_conv_adc_prescale(prescale);
    uint32_t opencm_adc = _abst_opencm_adc_conv(adc_n);
    uint32_t opencm_rcc_adc = _abst_opencm_rcc_adc_conv(adc_n);

    rcc_periph_clock_enable(opencm_rcc_adc);

    adc_power_off(opencm_adc);

#ifdef STM32F1
    rcc_set_adcpre(opencm_presc);

    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);

    adc_disable_scan_mode(opencm_adc);

    adc_set_single_conversion_mode(opencm_adc);
#endif // STM32F1

#ifdef STM32F4
    adc_set_clk_prescale(opencm_presc);

    adc_disable_scan_mode(opencm_adc);

    adc_set_single_conversion_mode(opencm_adc);

    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
#endif // STM32F4
    return ABST_OK;
}

/**
 * Helper function for initializing ADC channel based on information from :c:type:`abst_pin`.
 *
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 */
enum abst_errors _abst_init_adc_channel(const struct abst_pin *pin_ptr)
{
    uint32_t opencm_adc = _abst_opencm_adc_conv(pin_ptr->adc_num);
    uint32_t opencm_sample_time = _abst_conv_adc_samle_time(pin_ptr->adc_sample_time);

    uint32_t opencm_rcc_adc = _abst_opencm_rcc_adc_conv(pin_ptr->adc_num);

    rcc_periph_clock_enable(opencm_rcc_adc);

    adc_set_sample_time(opencm_adc, pin_ptr->adc_channel, opencm_sample_time);
    
    adc_power_on(opencm_adc);

#ifdef STM32F1 // Extra settings for STM32F1
    
    adc_enable_external_trigger_regular(opencm_adc, ADC_CR2_EXTSEL_SWSTART);
    
//     adc_reset_calibration(opencm_adc);
//     
//     adc_calibrate(opencm_adc);
#endif // STM32F1

    return ABST_OK;
}

/**
 * Read analog value from the pin via the Analog to Digital Converter
 *
 * :param pin_ptr: Pointer to :c:type:`abst_pin` with filled parameters.
 * :return: Read value (12 bit)
 */
uint16_t abst_adc_read(struct abst_pin *pin_ptr)
{
    if (pin_ptr == NULL)
        return 0;

    uint32_t opencm_adc = _abst_opencm_adc_conv(pin_ptr->adc_num);

#ifdef STM32F4 // Apparat resolution
    adc_set_resolution(opencm_adc, _abst_conv_adc_resolution(pin_ptr->adc_resolution));
#endif // STM32F4

    uint8_t channels[] = {pin_ptr->adc_channel};
    adc_set_regular_sequence(opencm_adc, 1, channels);

    adc_start_conversion_regular(opencm_adc);

    while (!adc_eoc(opencm_adc));

    uint16_t result = adc_read_regular(opencm_adc);
    
#ifdef STM32F4
    return result;
#endif // STM32F4


#ifdef STM32F1 // Software resolution
    switch (pin_ptr->adc_resolution) {
        case ABST_ADC_RES_12BIT:
            return result >> 0;
            break;
        case ABST_ADC_RES_10BIT:
            return result >> 2;
            break;
        case ABST_ADC_RES_8BIT:
            return result >> 4;
            break;
        case ABST_ADC_RES_6BIT:
            return result >> 6;
            break;
        default:
            return result;
    }
#endif // STM32F1
}

/**
 * Helper function for initializing an ADC in scan mode with setting up DMA.
 * After initializing an ADC is turned on.
 * Called from :c:func:`abst_adc_read_cont`
 *
 * :param adc_n: Number of ADC (1-3). If specified another the function will return :c:member:`abst_errors.ABST_WRONG_PARAMS`
 * :param adc_presc: ADC prescaler: 2, 4, 6, 8. If specified another the function will return :c:member:`abst_errors.ABST_WRONG_PARAMS`
 * :return: Error code according to :c:type:`abst_errors`.
 */
static enum abst_errors _abst_adc_init_scan_mode(uint8_t adc_n, uint8_t adc_presc)
{
    if (adc_n < 1 || adc_n > 3)
        return ABST_WRONG_PARAMS;

    uint32_t opencm_presc = _abst_conv_adc_prescale(adc_presc);
    uint32_t opencm_adc = _abst_opencm_adc_conv(adc_n);
    uint32_t opencm_rcc_adc = _abst_opencm_rcc_adc_conv(adc_n);

    rcc_periph_clock_enable(opencm_rcc_adc);

    adc_power_off(opencm_adc);

#ifdef STM32F1
    rcc_set_adcpre(opencm_presc);

    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);

    adc_enable_scan_mode(opencm_adc);

    adc_set_continuous_conversion_mode(opencm_adc);
#endif // STM32F1

#ifdef STM32F4
    adc_set_clk_prescale(opencm_presc);

    adc_set_continuous_conversion_mode(opencm_adc);
    
    adc_enable_scan_mode(opencm_adc);

    adc_eoc_after_each(opencm_adc);

    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
#endif // STM32F4

    return ABST_OK;
}

/**
 * Setup an ADC for **continuous** measurments with using DMA.
 * Values from ADC will be automatically updated in specified variables as soon as ADC read them.
 * 
 * 
 * :param pins_arr: Array of pointers to :c:type:`abst_pin`.
 * :param dest_arr: Array of **16-bit** integers where value will be put.
 * :param N: Length of arrays.
 * :param prescale: Prescale: 2, 4, 6, 8. If specified another the function will return :c:member:`abst_errors.ABST_WRONG_PARAMS`
 * :param prior: Priority of DMA requests.
 *     * 0 - LOW
 *     * 1 - MEDIUM
 *     * 2 - HIGH
 *     * 3 - VERY HIGH
 * :return: Error code according to :c:type:`abst_errors`.
 *
 * :c:member:`abst_pin.adc_num` and :c:member:`abst_pin.adc_resolution` 
 * have to be the same in all elements of array, otherwise :c:member:`ABST_WRONG_PARAMS` will be returned.
 *
 * Note: In STM32F1 :c:member:`abst_pin.adc_resolution` will be ignored 
 * as there no hardware resulution limiter.
 */
enum abst_errors abst_adc_read_cont(struct abst_pin *pins_arr[], 
                                    uint16_t dest_arr[], 
                                    uint8_t N, 
                                    uint8_t prescale, 
                                    uint8_t prior)
{
    if (pins_arr == NULL || dest_arr == NULL || N == 0)
        return ABST_WRONG_PARAMS;

    for (uint8_t i = 1; i < N; i++) {
        if (pins_arr[i]->adc_num != pins_arr[0]->adc_num)
            return ABST_WRONG_PARAMS;
        else if (pins_arr[i]->adc_resolution != pins_arr[0]->adc_resolution)
            return ABST_WRONG_PARAMS;
    } 

    enum abst_errors status = _abst_adc_init_scan_mode(pins_arr[0]->adc_num, prescale);
    if (status != ABST_OK)
        return status;
    
    for (uint8_t i = 0; i < N; i++) {
        abst_gpio_init(pins_arr[i]);
    }

    uint32_t opencm_adc = _abst_opencm_adc_conv(pins_arr[0]->adc_num);

#ifdef STM32F4 // Hardware resolution
    adc_set_resolution(opencm_adc, _abst_conv_adc_resolution(pins_arr[0]->adc_resolution));
#endif // STM32F4

    uint8_t channels[N];
    for (uint8_t i = 0; i < N; i++) {
        channels[i] = pins_arr[i]->adc_channel;
    }

    adc_set_regular_sequence(opencm_adc, N, channels);

    // DMA
    adc_enable_dma(opencm_adc);
#ifdef STM32F4
    adc_set_dma_continue(opencm_adc);
#endif // STM32F4

#ifdef STM32F1
    // STM32F1 Reference manual p.282

    enum abst_dma dma;
    uint8_t stream = 0; // Don't care
    uint8_t channel;
    switch (pins_arr[0]->adc_num) {
        case 1:
            dma = ABST_DMA_1;
            channel = 1;
            break;
        case 3:
            // ADC3 requests are available only in high-density and XL-density devices.
            dma = ABST_DMA_2;
            channel = 5; 
            break;
        default:
            return ABST_WRONG_PARAMS;
    }
    uint16_t *src_addr = &ADC_DR(opencm_adc);
#endif // STM32F1

#ifdef STM32F4
    // STM32F4 Reference manual p.308
    enum abst_dma dma = ABST_DMA_2;

    uint8_t stream;
    uint8_t channel;
    switch (pins_arr[0]->adc_num) {
        case 1:
            stream = 0;
            channel = 0;
            break;
        case 2:
            stream = 2;
            channel = 1;
            break;
        case 3:
            stream = 0;
            channel = 1;
            break;
        default:
            return ABST_WRONG_PARAMS;
    }
    uint16_t *src_addr = &ADC_DR(opencm_adc);
#endif // STM32F4
    enum abst_errors err = abst_init_dma_p2m_direct(dma, 
                                                    stream, 
                                                    channel, 
                                                    src_addr, 
                                                    dest_arr,
                                                    prior,
                                                    N,
                                                    ABST_DMA_SxCR_PSIZE_16BIT);

    if (err != ABST_OK)
        return err;
    
    adc_power_off(opencm_adc);
    abst_dma_start(dma, stream, channel);
    
    adc_power_on(opencm_adc);
    adc_start_conversion_regular(opencm_adc);
    
    return ABST_OK;
}
