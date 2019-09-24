
set(CPU_TYPE cortex-m4)

set(CPU_NAME STM32F407ZG)

set(LDSCRIPT ${CMAKE_CURRENT_LIST_DIR}/STM32F407ZG_FLASH.ld)


list(APPEND LIBS stm32f4)
list(APPEND LIBS lwip)
list(APPEND LIBS fatfs)


list(APPEND DEFINES STM32)
list(APPEND DEFINES STM32F4)

list(APPEND DEFINES LCD_DRIVER_ID=9341)

list(APPEND INCS ${CMAKE_CURRENT_LIST_DIR})

include(${CMAKE_CURRENT_LIST_DIR}/../stm32_config.cmake)

