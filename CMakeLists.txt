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
# /add
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(SIZE arm-none-eabi-size)
# add/

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Project)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")        # 添加gcc

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

#####################添加C++语言，与”Core project settings“调换位置
# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)
# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

###############################以下为修改过的cmakelist#########################################

function(USER_ADD_DIR dir)
    file(GLOB USER_SOURCE
            ${dir}/*.h
            ${dir}/*.c
            ${dir}/*.hpp
            ${dir}/*.cpp
            ${dir}/*.cc
    )
    target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${USER_SOURCE})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${dir})
endfunction()
###############################在此处添加你的文件路径
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/fifo)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/filter_alg)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/filter_alg)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/kalman_filter)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/lqr)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/PID)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/quaternion_kalman)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/user_maths)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/Algorithm/user_maths/cpp_files)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/app/safe_task)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/bsp/usart)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/bsp/can_part)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/bsp/can_part/can)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/bsp/dwt)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/bsp/spi)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/dr16)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/imu)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/motor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/motor/DJImotor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/motor/DMmotor)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/motor/DMmotor/dm_mit_mode)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/motor/DMmotor/dm_speed_mode)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/motor/DMmotor/dm_ls_mode)
#USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/referee)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/vision)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF/module/Rc)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/UAV_MainCode)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/UAV_MainCode/cmd)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/UAV_MainCode/debug_task)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/UAV_MainCode/fire)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/UAV_MainCode/gimbal)

#target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
#        stm32cubemx
#        ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a
#        ##################################添加数学库
#) 新版CubeMX有添加这个数学库

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
        COMMENT "Building hex & bin file...\nEXCUTABLE SIZE:"
        COMMAND ${SIZE} ${PROJECT_NAME}.elf
)
#set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--no-warn-rwx-segments")
