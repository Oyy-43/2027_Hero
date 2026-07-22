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
set(CMAKE_PROJECT_NAME Hero_2027)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

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
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Basic/alg_basic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/PID/alg_pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Filter/EKF/alg_filter_ekf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Filter/Frequency/alg_filter_frequency.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Matrix/alg_matrix.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/DataTrans/alg_transData.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/SingWave/alg_sin.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/CAN/drv_can.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/SPI/drv_spi.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/UART/drv_uart.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/USB/drv_usb.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/ADC/drv_adc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/System/Timestamp/sys_timestamp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Key/bsp_key.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Power/bsp_power.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/WS2812/bsp_ws2812.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Task/tsk_config_and_callback.cpp
)

# Add include paths 
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Basic
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/PID
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Filter/EKF
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Filter/Frequency
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Filter/Kalman
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Matrix
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/DataTrans
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/SingWave
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/CAN
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/SPI
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/UART
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/USB
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/ADC
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/System/Timestamp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Key
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Power
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/WS2812
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Task
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
