cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FreeRtos)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # FreeRTOS 各任务调用 User/app 里的应用层代码。
    User/app/app.c

    # OLED 使用软件 I2C，所以 OLED 驱动和软件 I2C 驱动都要参与编译。
    User/bsp/bsp_oled.c
    User/bsp/bsp_soft_i2c.c

    # 应用层通过这个 BSP 读取 TIM4 编码器。
    User/bsp/bsp_encoder.c

    # 应用层还会使用 TB6612 电机驱动和低电平有效按键。
    User/bsp/bsp_motor.c
    User/bsp/bsp_key.c

    # VOFA 串口波形调试输出。
    User/bsp/bsp_vofa.c

    # 速度闭环使用的 PID 算法模块。
    User/algorithm/pid.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 让应用层文件可以直接包含 "app.h"、"bsp_oled.h" 等头文件。
    User/app
    User/bsp
    User/algorithm
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
