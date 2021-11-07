#ifndef _MOD_SENSORS_QSPI_RAM_H_
#define _MOD_SENSORS_QSPI_RAM_H_

#include <zephyr/types.h>
#include <device.h>
#include <errno.h>

#define QSPI_RAM_MODE_MEM_MAPPED         0
#define QSPI_RAM_MODE_INDIRECT           1
//typedef int (*qspi_ram_api_configure_t)(const struct device *dev, const struct qspi_ram_config config);

typedef int (*qspi_ram_api_set_access_mode_t)(const struct device *dev, uint8_t mode);
typedef int (*qspi_ram_api_get_access_mode_t)(const struct device *dev);

typedef int (*qspi_ram_api_write_mem_addr_t)(const struct device *dev, uint32_t addr, uint8_t data);

typedef int (*qspi_ram_api_read_mem_addr_t)(const struct device *dev, uint32_t addr, uint8_t *data);

typedef int (*qspi_ram_api_write_mem_range_t)(const struct device *dev, uint32_t addr, uint8_t *data, uint32_t data_len);

typedef int (*qspi_ram_api_read_mem_range_t)(const struct device *dev);


__subsystem struct qspi_ram_driver_api {
    qspi_ram_api_set_access_mode_t set_access_mode;
    qspi_ram_api_get_access_mode_t get_access_mode;
    qspi_ram_api_write_mem_addr_t write_mem_addr;
    qspi_ram_api_read_mem_addr_t read_mem_addr;
    qspi_ram_api_write_mem_range_t write_mem_range;
    qspi_ram_api_read_mem_range_t read_mem_range;
};

__syscall int qspi_ram_set_access_mode(const struct device *dev, uint8_t mode);

static inline int z_impl_qspi_ram_set_access_mode(const struct device *dev, uint8_t mode)
{
    const struct qspi_ram_driver_api *api = (const struct qspi_ram_driver_api *)dev->api;
    return api->set_access_mode(dev, mode);
}

__syscall int qspi_ram_get_access_mode(const struct device *dev);

static inline int z_impl_qspi_ram_get_access_mode(const struct device *dev)
{
    const struct qspi_ram_driver_api *api = (const struct qspi_ram_driver_api *)dev->api;
    return api->get_access_mode(dev);
}

__syscall int qspi_ram_write_mem_addr(const struct device *dev, uint32_t addr, uint8_t data);

static inline int z_impl_qspi_ram_write_mem_addr(const struct device *dev, uint32_t addr, uint8_t data)
{
    const struct qspi_ram_driver_api *api = (const struct qspi_ram_driver_api *)dev->api;
    return api->write_mem_addr(dev, addr, data);
}

__syscall int qspi_ram_read_mem_addr(const struct device *dev, uint32_t addr, uint8_t *data);

static inline int z_impl_qspi_ram_read_mem_addr(const struct device *dev, uint32_t addr, uint8_t *data)
{
    const struct qspi_ram_driver_api *api = (const struct qspi_ram_driver_api *)dev->api;
    return api->read_mem_addr(dev, addr, data);
}

__syscall int qspi_ram_write_mem_addr_range(const struct device *dev, uint32_t addr, uint8_t *data, uint32_t data_len);

static inline int z_impl_qspi_ram_write_mem_addr_range(const struct device *dev, uint32_t addr, uint8_t *data, uint32_t data_len)
{
    const struct qspi_ram_driver_api *api = (const struct qspi_ram_driver_api *)dev->api;
    return api->write_mem_range(dev, addr, data, data_len);
}

__syscall int qspi_ram_read_mem_addr_range(const struct device *dev);

static inline int z_impl_qspi_ram_read_mem_addr_range(const struct device *dev)
{
    const struct qspi_ram_driver_api *api = (const struct qspi_ram_driver_api *)dev->api;
    return api->read_mem_range(dev);
}


#include <syscalls/qspi_ram.h>
#endif