cmake_minimum_required(VERSION 3.20)
project(moto_control LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── spdlog ────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.13.0
)
FetchContent_MakeAvailable(spdlog)

# ── libmodbus (static) ────────────────────────────────
add_library(modbus STATIC
    include/libmodbus/modbus.c
    include/libmodbus/modbus-data.c
    include/libmodbus/modbus-rtu.c
    include/libmodbus/modbus-tcp.c
    include/libmodbus/modbus.h
    include/libmodbus/modbus-rtu.h
    include/libmodbus/modbus-rtu-private.h
    include/libmodbus/modbus-tcp.h
    include/libmodbus/modbus-tcp-private.h
    include/libmodbus/modbus-private.h
    include/libmodbus/modbus-version.h
    include/libmodbus/config.h
)
target_include_directories(modbus PUBLIC include/libmodbus)
target_compile_definitions(modbus PUBLIC MODBUS_STATIC)
if(WIN32)
    target_link_libraries(modbus PUBLIC ws2_32)
endif()

# ── executable ────────────────────────────────────────
add_executable(moto_control
    src/main.cpp
    src/control/ChuangRui_Control.cpp
    src/control/ChuangRui_Control.h
    src/control/MotionControl.h
    include/modbus.h
    include/json.hpp
)
target_include_directories(moto_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(moto_control PRIVATE modbus spdlog::spdlog)

# copy config.json to output
add_custom_command(TARGET moto_control POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/config.json"
        "$<TARGET_FILE_DIR:moto_control>/config.json"
)

# MSVC: UTF-8 source encoding
if(MSVC)
    target_compile_options(moto_control PRIVATE /utf-8)
    target_compile_options(modbus PRIVATE /utf-8)
endif()
