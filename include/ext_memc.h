#ifndef _EXT_MEMC_H
#define _EXT_MEMC_H

#include <zephyr.h>

/**
 * This module implements a simple buffer-based memory controller. It allows 
 * an application to allocate and access contiguous blocks of memory provided
 * by an external SRAM device. 
 * 
 * It should be noted that currently, this module makes exclusive use of 
 * the qspi_ram driver with a memory device attached to a qspi bus and 
 * supporting single address and address range reads/writes.
 * 
 * An auxilliary memory-mapped mode allows defined memory blocks to be 
 * accessed in a read-only manner, as if they were part of the system address 
 * space. It should be noted that this mode must be supported by the 
 * application's hardware platform and should be implemented by the platform-
 * specific qspi_ram driver. To enable this mode CONFIG_QSPI_RAM_MEM_MAPPED
 * must be enabled to compile the relevant parts of this module /and/ those
 * of the qspi_ram driver.
 */

#define EXT_MEMC_MODE_RD_WR             0x00
#define EXT_MEMC_MODE_MEM_MAPPED        0x01

struct ext_memc_cfg {
    uint32_t mem_size;
    uint32_t mem_mapped_start_addr;
};

typedef struct  {
    uint32_t mem_start_addr;          // start of memory in device
    volatile uint8_t *buf;                         // mem-mapped start of memory blk
    uint32_t block_len;               // total length in bytes
    uint32_t w_ptr;                         // data ptr for append writes
    uint32_t r_ptr;                         // data ptr for range reads
} ext_memc_blk_t;

/**
 * @brief Initialises the memory controller.
 * 
 * @param qspi_dev Pointer to a qspi_ram device compatible to which the memory has been 
 *        connected.
 * @param cfg Configuration structure for this module to setup the desired 
 *        memory size and other essential options.
 * 
 * @retval 0 if init was okay.
 * @retval -errno if there was a failure to setup the memory controller. Most 
 *         likely to be related to the qspi device.
 */
int ext_memc_init(const struct device *qspi_dev, struct ext_memc_cfg cfg);

/**
 * @brief Set the current operating mode for memory access.
 * 
 * @param mode Desired mode - either read-only memory mapped or indirect
 *        read/write.
 * @note These modes are based on those provided by the ST QSPI memory 
 *       peripheral.
 * 
 * @retval 0 if mode change was okay.
 * @retval -errno if there was an error.
 */    
int ext_memc_set_mode(uint8_t mode);

/**
 * @brief Allocate a new block of memory in an interface style similar to that
 *        of malloc()
 * 
 * @param block Pointer to an ext_memc_block structure where info about the 
 *        newly allocated block of memory.
 * @param site Desired size of the new memory block in bytes.
 * 
 * @retval 0 if allocation was okay.
 * @retval -errno if there was a problem.
 *         @note block will also be NULL if there was an allocation error.
 */
ext_memc_blk_t * ext_memc_allocate_block(size_t size);

/**
 * @brief Free and delete a block of previously allocated memory block.
 * 
 * @param block Pointer to a currently allocated memory block.
 * 
 * @retval 0 if de-allocation was okay.
 * @retval -errno if there was a problem.
 */
int ext_memc_free_block(ext_memc_blk_t *block);

/**
 * @brief Write to an allocated block of data.
 * 
 * @note This function will write starting at the current location of 
 *       w_ptr and increment it based on how much data is written.
 *       The return value will also indicate whether an overflow occurred.
 *       When last_batch is true, the w_ptr will be reset after the write
 *       has been completed.
 * 
 * @param block
 * @param data
 * @param data_len Length
 * 
 * @retval 0 if write was okay.
 * @retval -errno if there was a problem.
 */
int ext_memc_write_append_to_block(ext_memc_blk_t *block, uint8_t *data, 
                            size_t data_len, bool last_batch);

/**
 * @brief Write to an allocated block of data.
 * 
 * @note This function will always write from the block's start address.
 * 
 * @param block
 * @param data
 * @param data_len Length
 * 
 * @retval 0 if write was okay.
 * @retval -errno if there was a problem.
 */
int ext_memc_write_to_block(ext_memc_blk_t *block, uint8_t *data,
                            size_t data_len);
#endif