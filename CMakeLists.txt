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
set(CMAKE_PROJECT_NAME Chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

option(CHASSIS_ENABLE_EXPERIMENTS "Build PRBS/FFID/wheeltest experiment code" OFF)
message("Experiments enabled: " ${CHASSIS_ENABLE_EXPERIMENTS})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

function(include_sub_directories_recursively target root_dir)
    if (IS_DIRECTORY ${root_dir})
        target_include_directories(${target} PRIVATE ${root_dir})
    endif()

    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*)
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${target} ${root_dir}/${sub})
        endif()
    endforeach()
endfunction()

file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/application/*.c"
    "${CMAKE_SOURCE_DIR}/bsp/*.c"
    "${CMAKE_SOURCE_DIR}/modules/*.c"
)

list(FILTER USER_SOURCES EXCLUDE REGEX ".*/application/cmd/srv_nav\\.c$")

if(NOT CHASSIS_ENABLE_EXPERIMENTS)
    list(FILTER USER_SOURCES EXCLUDE REGEX ".*/application/chassis/sysid_shared\\.c$")
    list(FILTER USER_SOURCES EXCLUDE REGEX ".*/application/chassis/sysid_prbs\\.c$")
    list(FILTER USER_SOURCES EXCLUDE REGEX ".*/application/chassis/sysid_ffid\\.c$")
    list(FILTER USER_SOURCES EXCLUDE REGEX ".*/application/chassis/sysid_wheeltest\\.c$")
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

include_sub_directories_recursively(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/application)
include_sub_directories_recursively(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/bsp)
include_sub_directories_recursively(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/modules)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    APP_CFG_ENABLE_EXPERIMENTS=$<IF:$<BOOL:${CHASSIS_ENABLE_EXPERIMENTS}>,1,0>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin"
)
