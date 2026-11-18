# The root CMakeLists.txt owns application and HAL source collection.
# Keep this target interface-only to avoid duplicate sources and premature
# references to the executable before add_executable() is called.
cmake_minimum_required(VERSION 3.22)
add_library(stm32cubemx INTERFACE)
target_compile_definitions(stm32cubemx INTERFACE
    USE_HAL_DRIVER
    STM32F427xx
    $<$<CONFIG:Debug>:DEBUG>
)
target_include_directories(stm32cubemx INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/CMSIS/Include
)