#include <errno.h>

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include <drivers/qspi_ram.h>
#include <ext_memc.h>

#include <stdlib.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ext_memc, CONFIG_EXT_MEMC_LOG_LEVEL);

#define BLK_FLG_STATUS                  0x01
#define BLK_FLG_COLLECT                 0x02

#define BLK_STATUS_ALLOCATED            0x00
#define BLK_STATUS_FREE                 0x01

typedef struct mem_blk mem_blk_t;
struct mem_blk{
    ext_memc_blk_t *mem_blk;
    uint8_t flags;
    mem_blk_t *prev;
    mem_blk_t *next;
};

struct mem_blk_list {
    mem_blk_t *head;
    mem_blk_t *curr;
    uint32_t len;
};

struct ext_memc_data {
    const struct device *qspi;
    struct ext_memc_cfg cfg;
    struct mem_blk_list mem;
    uint8_t mode;    
};

static struct ext_memc_data memc_data;

/**
 * Iterate the list until we find the mem_blk that holds the given memc_blk
 */
static int memc_blk_to_mem_blk(struct mem_blk_list *blk_list, 
                               ext_memc_blk_t *memc_blk)
{
    blk_list->curr = blk_list->head;
    while (blk_list->curr != NULL)
    {
        if (blk_list->curr->mem_blk->mem_start_addr == memc_blk->mem_start_addr)
        {
            return 0;
        }
        else
        {
            blk_list->curr = blk_list->curr->next;
        }
    }
    return -EINVAL;
}

/**
 * Iterate over the mem_blk_list and combine any blocks with the FREE flag set.
 */
static int consolidate_free_blks(struct mem_blk_list *blk_list)
{

    // this will remove the curr item from mem_blk_list
    /*
    mem_blk_t *tmp = &memc_data.mem->curr;

    // if its the head being removed, setup the new head
    if (memc_data.mem->head == memc_data.mem->curr)
        memc_data.mem->head = tmp->next;

    // make prev member of next item point to the one before tmp
    if (tmp->next != NULL)
        tmp->next->prev = tmp->prev;

    // make the next member previous item point to the one after tmp
    if (tmp-prev != NULL)
        tmp->prev->next = tmp->next;
    */
   return 0;
}

static int get_free_sized_mem_blk(struct mem_blk_list *blk_list, size_t size)
{
    blk_list->curr = blk_list->head;
    while (blk_list->curr != NULL)
    {
        if ((size_t)blk_list->curr->mem_blk->block_len >= size)
        {
            return 0;
        }
        else
        {
            blk_list->curr = blk_list->curr->next;
        }
    }
    return -EINVAL;
}

static int write_chunk(uint32_t addr, uint8_t *data, uint32_t data_len)
{
    int ret;

    ret = qspi_ram_write_mem_addr_range(memc_data.qspi,
                        addr,
                        data,
                        data_len);
    if (ret != 0)
    {
        LOG_ERR("%d: could not write memory chunk of size %d to addr 0x%x", 
                    ret,
                    data_len,
                    addr);
        return ret;
    }

    // LOG_DBG("wrote %d byte chunk to mem at addr %x", 
    //                     data_len,
    //                     addr);
    // for (int i = 0; i < data_len; i++)
    // {
    //     LOG_DBG("mem addr %x=%x", addr+i, data[i]);
    // }
    return 0;
}

int ext_memc_init(const struct device *qspi_dev, struct ext_memc_cfg cfg)
{
    memc_data.qspi = qspi_dev;
    if (memc_data.qspi == NULL)
    {
        LOG_ERR("could not get or start qspi device");
        return -EINVAL;
    }
    memc_data.cfg.mem_size = cfg.mem_size;
    memc_data.cfg.mem_mapped_start_addr = cfg.mem_mapped_start_addr;

    ext_memc_set_mode(EXT_MEMC_MODE_RD_WR);

    memc_data.mem.head = NULL;
    memc_data.mem.curr = NULL;
    memc_data.mem.len = 0;

    mem_blk_t *init_blk = malloc(sizeof(mem_blk_t));
    ext_memc_blk_t *init_mem = malloc(sizeof(ext_memc_blk_t));

    init_blk->flags = (BLK_STATUS_FREE << BLK_FLG_STATUS) | 
                     (0 << BLK_FLG_COLLECT);
    
    init_blk->prev = NULL;
    init_blk->next = NULL;
    init_mem->mem_start_addr = 0x00000;
    init_mem->block_len = memc_data.cfg.mem_size;
    init_mem->buf = (uint32_t *)memc_data.cfg.mem_mapped_start_addr;
    init_blk->mem_blk = init_mem;
    memc_data.mem.head = init_blk;
    memc_data.mem.curr = init_blk;
    memc_data.mem.len = 1;

    return 0;
}

int ext_memc_set_mode(uint8_t mode)
{
    int ret;
    uint8_t trans_mode = QSPI_RAM_MODE_INDIRECT;
    if (mode == EXT_MEMC_MODE_RD_WR)
        trans_mode = QSPI_RAM_MODE_INDIRECT;
    else if (mode == EXT_MEMC_MODE_MEM_MAPPED)
        trans_mode = QSPI_RAM_MODE_MEM_MAPPED;
    else 
    {
        LOG_ERR("invalid access mode specified");
        return -EINVAL;
    }

    ret = qspi_ram_set_access_mode(memc_data.qspi, trans_mode);
    if (ret != 0)
    {
        LOG_ERR("could not set qspi_ram access mode");
        return ret;
    }
    LOG_DBG("set access mode to %d", mode);
    memc_data.mode = mode;
    return ret;
}

ext_memc_blk_t * ext_memc_allocate_block(size_t size)
{
    // 1.0 find a free block of memory
    // 2.0 check if size <= that of free memory block
    // 2.1 malloc a new memc_blk_t descriptor 
    //     --> set start addr, block_len, and buf addr
    //     --> create new free block if required, for additional free memory
    //              calc start addr of new blk, set size, buf addr
    //      
    // 2.2 malloc a new mem_blk container
    //     --> add memc_blk_t and set flags
    // 2.3 insert new mem_blk container into mem_blk_list
    // 
    // 3.0 test and inevitably find out that you fucked up...

    int ret;

    ret = get_free_sized_mem_blk(&memc_data.mem, size);
    if (ret != 0)
    {
        LOG_ERR("failed to allocate block of size %d bytes", size);
        return NULL;
    }
    ext_memc_blk_t *new_blk = malloc(sizeof(ext_memc_blk_t));
    new_blk->mem_start_addr = memc_data.mem.curr->mem_blk->mem_start_addr;
    new_blk->buf = (uint32_t *)(memc_data.cfg.mem_mapped_start_addr +
                    memc_data.mem.curr->mem_blk->mem_start_addr);
    new_blk->block_len = size;
    new_blk->w_ptr = 0;
    new_blk->r_ptr = 0;

    mem_blk_t *new_mem_blk = malloc(sizeof(mem_blk_t));
    new_mem_blk->mem_blk = new_blk;
    new_mem_blk->flags = (BLK_STATUS_ALLOCATED << BLK_FLG_STATUS);
    new_mem_blk->prev = memc_data.mem.curr->prev;

    LOG_DBG("mem addr %d %x %x", memc_data.mem.curr->mem_blk->mem_start_addr,
                memc_data.cfg.mem_mapped_start_addr,
                new_blk->block_len);

    if (size < memc_data.mem.curr->mem_blk->block_len)
    {
        ext_memc_blk_t *rem_blk = malloc(sizeof(ext_memc_blk_t));
        rem_blk->mem_start_addr = new_blk->mem_start_addr+new_blk->block_len;
        rem_blk->buf = (uint32_t *)(memc_data.cfg.mem_mapped_start_addr +
                        rem_blk->mem_start_addr);
        rem_blk->block_len = memc_data.mem.curr->mem_blk->block_len-new_blk->block_len;
        rem_blk->w_ptr = 0;
        rem_blk->r_ptr = 0;

        mem_blk_t *rem_mem_blk = malloc(sizeof(mem_blk_t));
        rem_mem_blk->mem_blk = rem_blk;
        rem_mem_blk->flags = (BLK_STATUS_FREE << BLK_FLG_STATUS);
        rem_mem_blk->prev = new_mem_blk;
        
        new_mem_blk->next = rem_mem_blk;
        rem_mem_blk->next = memc_data.mem.curr->next;
    } 
    // else
    // {
    //     new_mem_blk->next = memc_data.mem.curr->next;
    //     free(memc_data.mem.curr);
    // }
    if (&memc_data.mem.head == &memc_data.mem.curr)
    {
        free(memc_data.mem.curr);
        memc_data.mem.curr = new_mem_blk;
        memc_data.mem.head = new_mem_blk;
    }
    else 
    {
        free(memc_data.mem.curr);
        memc_data.mem.curr = new_mem_blk;
    }

    return new_blk;
}

int ext_memc_free_block(ext_memc_blk_t *block)
{
    int ret;

    if (memc_data.mem.len < 2)
    {
        return -EINVAL;
    }

    ret = memc_blk_to_mem_blk(&memc_data.mem, block);
    if (ret != 0)
    {
        LOG_ERR("could not find memc block to free");
        return ret;
    }

    // free the actual memc_blk_t descriptor
    free(block);
    // mark this block free and flag for collection to combine with adjacent free blocks
    memc_data.mem.curr->flags |= (BLK_STATUS_FREE << BLK_FLG_STATUS) | 
                                 (1 << BLK_FLG_COLLECT);
    consolidate_free_blks(&memc_data.mem);

    return 0;
}

int ext_memc_write_append_to_block(ext_memc_blk_t *block, uint8_t *data, 
                            size_t data_len, bool last_batch)
{
    int ret;

    if (memc_data.qspi != NULL)
    {
        // if the access mode isn't set up properly, we need to change it
        if (memc_data.mode != EXT_MEMC_MODE_RD_WR)
        {
            ret = ext_memc_set_mode(EXT_MEMC_MODE_RD_WR);
            if (ret != 0)
                return ret;
        }

        // if we've gone past the end of the block
        if ((block->w_ptr + data_len) > block->block_len)
        {
            LOG_DBG("block overrun");
            ret = write_chunk(block->mem_start_addr + block->w_ptr,
                                data,
                                block->block_len - block->w_ptr);
            if (ret != 0)
                return ret;

            ret = write_chunk(block->mem_start_addr, 
                                data+(block->block_len - block->w_ptr),
                                data_len - (block->block_len - block->w_ptr));
            block->w_ptr = data_len - (block->block_len - block->w_ptr);
        }
        else 
        {
            LOG_DBG("block normal append");
            ret = write_chunk(block->mem_start_addr + block->w_ptr,
                                data,
                                data_len);
            if (ret != 0)
                return ret;
            
            // update w_ptr based on data_len
            block->w_ptr += data_len;
        }

        if (last_batch)
        {
            block->w_ptr = 0;
        }
        return 0;
    }
    return -EINVAL;

}

int ext_memc_write_to_block(ext_memc_blk_t *block, uint8_t *data,
                            size_t data_len)
{
    int ret;

    if (memc_data.qspi != NULL)
    {
        // if the access mode isn't set up properly, we need to change it
        if (memc_data.mode != EXT_MEMC_MODE_RD_WR)
        {
            ret = ext_memc_set_mode(EXT_MEMC_MODE_RD_WR);
            if (ret != 0)
                LOG_ERR("failed to enter RD_WR mode");
                return ret;
        }

        ret = write_chunk(block->mem_start_addr + block->w_ptr,
                                data,
                                data_len);
        if (ret != 0)
            return ret;
        
        // update w_ptr based on data_len if append writes are desired after
        block->w_ptr += data_len;

        return 0;
    }
    return -EINVAL;
}