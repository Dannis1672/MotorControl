cmake_minimum_required(VERSION 3.10)
project(moto_control LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Platform: pthread (Linux / Unix) ─────────────────────
if(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
endif()

# ── Boost (HTTP server) ───────────────────────────────
find_package(Boost REQUIRED COMPONENTS system)

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
target_compile_definitions(modbus PRIVATE _GNU_SOURCE)
if(WIN32)
    # Windows: Winsock for modbus TCP
    target_link_libraries(modbus PUBLIC ws2_32)
endif()
if(UNIX)
    # Linux / Unix: modbus RTU needs termios; TCP needs standard sockets
    target_link_libraries(modbus PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()

# ── executable ────────────────────────────────────────
add_executable(moto_control
    src/main.cpp
    src/control/ChuangRui_Control.cpp
    src/control/ChuangRui_Control.h
    src/control/ModbusClient.cpp
    src/control/ModbusClient.h
    src/control/MotionControl.h
    src/server/HttpServer.h
)
target_include_directories(moto_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(moto_control PRIVATE modbus spdlog::spdlog Boost::system)
if(UNIX AND NOT APPLE)
    target_link_libraries(moto_control PRIVATE Threads::Threads)
endif()

# copy config.json to output
add_custom_command(TARGET moto_control POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/config.json"
        "$<TARGET_FILE_DIR:moto_control>/config.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/assets/config.json"
        "${CMAKE_CURRENT_BINARY_DIR}/config.json"
)

# MSVC: UTF-8 source encoding
if(MSVC)
    target_compile_options(moto_control PRIVATE /utf-8)
    target_compile_options(modbus PRIVATE /utf-8)
else()
    # GCC / Clang: warn on common issues but don't be pedantic
    target_compile_options(moto_control PRIVATE -Wall -Wextra)
endif()

# ── Packaging (CPack + WiX MSI) ──────────────────────
# Install rules: these define what goes into the package
install(TARGETS moto_control RUNTIME DESTINATION .)
install(FILES assets/config.json DESTINATION .)

# Bundle MSVC redistributable DLLs alongside the exe (not in bin/)
if(WIN32)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION .)
    include(InstallRequiredSystemLibraries)
endif()

# CPack common settings
set(CPACK_PACKAGE_NAME "moto_control")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_VENDOR "MotorControl")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Motor Control Application")
set(CPACK_VERBATIM_VARIABLES TRUE)

set(CPACK_GENERATOR "ZIP")
include(CPack)