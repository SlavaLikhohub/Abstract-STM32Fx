#include "abstractFLASH.h"

#include <libopencm3/stm32/flash.h>
/**
 * Erase page of flash. Flash is locked after the operation.
 * 
 * :param start_address: Starg address of a page
 * :return: Error code according to :c:type:`abst_errors`.
 */
enum abst_errors abst_flash_erase_page(uint32_t start_address)
{
    flash_unlock();
    
    flash_erase_page(start_address);
    
    uint32_t flash_status = flash_get_status_flags();
    
    flash_lock();
    if(flash_status != FLASH_SR_EOP)
        return ABST_OPERATION_FAILED;
    else 
        return ABST_OK;
}

/**
 * Write into a flash
 * 
 * :param start_address: Start address of flash where data should be written
 * :param num_elements: Number of elements in array
 * :param input_data: Array of data to be written
 * :return: Error code according to :c:type:`abst_errors`.
 */
enum abst_errors abst_flash_write(uint32_t start_address, 
                                  uint16_t num_elements, 
                                  uint32_t input_data[])
{
    uint32_t flash_status = 0;
    
    flash_unlock();
    
    for (uint16_t i = 0; i < num_elements; i++) {
        flash_program_word(start_address + i, input_data[i]);
        
        flash_status = flash_get_status_flags();
        if(flash_status != FLASH_SR_EOP)
            return ABST_OPERATION_FAILED;
        
        if(*((uint32_t *)(start_address) + i) != input_data[i])
            return ABST_OPERATION_FAILED;
    }
    flash_lock();
    
    return ABST_OK;
}


/**
 * Read data from the FLASH
 * 
 * :param start_address: Start address of a FLASH where to read
 * :param num_elements: Number of elements to read
 * :param output_data: Array where to put data
 */
enum abst_errors abst_flash_read(uint32_t start_address, 
                                 uint16_t num_elements, 
                                 uint32_t output_data[])
{
    for (uint16_t i = 0; i < num_elements; i++) {
        output_data[i] = *((uint32_t *)(start_address) + i);
    }
    return ABST_OK;
}
