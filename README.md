# Zephyr External Memory Module

This repository is a collection of memory managment oriented drivers and 
subsystems for use with the [Zephyr Framework](https://docs.zephyrproject.org/latest/introduction/index.html) 
that have been created during and for my own Zephyr RTOS-based projects. 

### Drivers
- QSPI RAM (currently implemented for STM32 parts only)

### Subsystems
- QSPI RAM External memory controller
    - Sequentially allocates contiguous memory blocks
    - writes are done in 'indirect' mode 
    - reads can be done in memory-mapped or 'indirect' mode (limitation of
    STM32 QSPI interface)
