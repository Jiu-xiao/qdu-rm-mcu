cmake_minimum_required(VERSION 3.11)

set(BSP_DIR ${HW_DIR}/bsp)
set(MCU_DIR ${HW_DIR}/mcu)

if(CONFIG_BOARD_DEV_C)
    set(CMAKE_MODULE_PATH ${BSP_DIR}/rm-c)
else()
    message(FATAL_ERROR "No board selected.")
endif()

include(board)