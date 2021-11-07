# Find Zephyr. This also loads Zephyr's build system.

zephyr_include_directories(include)
list(APPEND SYSCALL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SYSCALL_INCLUDE_DIRS ${SYSCALL_INCLUDE_DIRS} PARENT_SCOPE)
add_subdirectory(drivers)
add_subdirectory(subsys)
