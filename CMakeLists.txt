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
add_executable(${CMAKE_PROJECT_NAME}
    # User_Files/3_App/Task/Chassis_Task/Chassis_Task.cpp
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# ---------------------------------------------------------------------------
# DSP include de-conflict (survives CubeMX regeneration)
# ---------------------------------------------------------------------------
# CubeMX re-adds the OLD CMSIS-DSP include dir (Middlewares/ST/ARM/DSP/Inc) to
# the stm32cubemx interface on every code generation. This project links the NEW
# precompiled DSP library under User_Files/1_Middleware/DSP instead, so including
# both header sets in one TU causes duplicate declarations (e.g. read_q15x2,
# write_q15x2_ia, arm_status / ARM_MATH_SUCCESS ...). Filter the stale dir out so
# only the new DSP headers are used.
get_target_property(_MX_INCLUDE_DIRS stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
if(_MX_INCLUDE_DIRS AND NOT _MX_INCLUDE_DIRS STREQUAL "_MX_INCLUDE_DIRS-NOTFOUND")
    list(FILTER _MX_INCLUDE_DIRS EXCLUDE REGEX "Middlewares/ST/ARM/DSP/Inc")
    set_target_properties(stm32cubemx PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_MX_INCLUDE_DIRS}")
endif()
unset(_MX_INCLUDE_DIRS)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Slope/alg_slope.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/CAN/drv_can.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/SPI/drv_spi.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/OSPI/drv_ospi.cpp    
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/UART/drv_uart.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/CRSF/crsf_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/CRSF/crsf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/USB/drv_usb.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/ADC/drv_adc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/WDG/drv_wdg.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/System/Timestamp/sys_timestamp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/FSM/FSM_Key/FSM_Key.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Key/bsp_key.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Power/bsp_power.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/WS2812/bsp_ws2812.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Buzzer/bsp_buzzer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/W25Q64JV/bsp_w25q64jv.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/W25Q64JV/bsp_w25q64jv_register.cpp        
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Chassis_Control/Steer_Chassis/steer_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Data_Transfer/App_DataTransfer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/Motor/Motor_DJI/drv_motor_dji.cpp   
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/Motor/Motor_DM/drv_motor_dm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/Motor/Motor_LK/drv_motor_lk.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Task/Chassis_Task/Chassis_Task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Task/Gimbal_Task/Gimbal_Task.cpp
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
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Algorithm/Slope
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/CAN
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/SPI
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/OSPI
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/UART
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/USB
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/ADC
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/WDG
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/Driver/CRSF
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/System/Timestamp
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/FSM
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Key
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Power
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/WS2812
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/Buzzer
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/BSP/W25Q64JV    
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Task/Chassis_Task
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Task/Gimbal_Task
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Chassis_Control
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Chassis_Control/Steer_Chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/3_App/Data_Transfer
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/Motor/Motor_DJI
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/Motor/Motor_DM
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/2_Device/Motor/Motor_LK

)

# DSP library include path (CubeMX-safe, not overwritten)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/DSP/Inc
)

# Precompiled DSP static library location (per build config)
set(DSP_PRECOMPILED_LIB ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/DSP/Lib/${CMAKE_BUILD_TYPE}/libDSP_Library.a)

# Option: use precompiled DSP library to save build time.
# If the .a exists it is linked directly and DSP sources are NOT compiled.
# Delete DSP/Lib/<config>/libDSP_Library.a or set -DDSP_USE_PRECOMPILED=OFF to rebuild from source.
option(DSP_USE_PRECOMPILED "Use precompiled DSP static library" ON)

if(DSP_USE_PRECOMPILED AND EXISTS ${DSP_PRECOMPILED_LIB})
    message(STATUS "DSP: using precompiled library ${DSP_PRECOMPILED_LIB}")
    add_library(DSP_Library STATIC IMPORTED)
    set_target_properties(DSP_Library PROPERTIES IMPORTED_LOCATION ${DSP_PRECOMPILED_LIB})
    target_include_directories(DSP_Library INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/DSP/Inc
    )
else()
    message(STATUS "DSP: compiling from source, result will be saved to ${DSP_PRECOMPILED_LIB}")
    # DSP library source files (exclude NEON/Helium arch-specific files)
    file(GLOB_RECURSE DSP_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/DSP/Source/*.c
    )
    list(FILTER DSP_SOURCES EXCLUDE REGEX "_neon|_helium|_mve|Functions\\.c$|FunctionsF16\\.c$|CommonTables\\.c$|CommonTablesF16\\.c$")

    # Build DSP once as a STATIC library so it can be reused on later builds
    add_library(DSP_Library STATIC ${DSP_SOURCES})
    target_include_directories(DSP_Library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/DSP/Inc)
    target_compile_definitions(DSP_Library PRIVATE ARM_MATH_CM7)
    target_link_libraries(DSP_Library PUBLIC stm32cubemx)

    # Save the built .a to the precompiled location so future builds skip DSP compilation
    add_custom_command(TARGET DSP_Library POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/User_Files/1_Middleware/DSP/Lib/${CMAKE_BUILD_TYPE}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:DSP_Library> ${DSP_PRECOMPILED_LIB}
        COMMENT "DSP: saved precompiled library to ${DSP_PRECOMPILED_LIB}"
    )
endif()

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    DSP_Library

    # Add user defined libraries
)
