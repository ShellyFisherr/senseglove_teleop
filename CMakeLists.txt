
cmake_minimum_required(VERSION 3.16)

project(senseglove_teleop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# SenseGlove SDK location
set(SENSEGLOVE_API
    "$ENV{HOME}/senseglove/SenseGlove-API"
)

# Linux x86-64 SDK libraries
set(SENSEGLOVE_LIB_DIR
    "${SENSEGLOVE_API}/lib/linux/v22/x86-64/release"
)

set(SGCORE_LIB
    "${SENSEGLOVE_LIB_DIR}/libsgcore.so"
)

set(SGCONNECT_LIB
    "${SENSEGLOVE_LIB_DIR}/libsgconnect.so"
)

# Verify SDK files
if(NOT EXISTS "${SGCORE_LIB}")
    message(FATAL_ERROR "Could not find libsgcore.so: ${SGCORE_LIB}")
endif()

if(NOT EXISTS "${SGCONNECT_LIB}")
    message(FATAL_ERROR "Could not find libsgconnect.so: ${SGCONNECT_LIB}")
endif()

find_package(Threads REQUIRED)

add_executable(senseglove_reader
    teleop.cpp
)

target_include_directories(senseglove_reader PRIVATE
    "${SENSEGLOVE_API}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(senseglove_reader PRIVATE
    "${SGCORE_LIB}"
    "${SGCONNECT_LIB}"
    Threads::Threads
)

# Find the SDK libraries at runtime without setting LD_LIBRARY_PATH
set_target_properties(senseglove_reader PROPERTIES
    BUILD_RPATH "${SENSEGLOVE_LIB_DIR}"
    INSTALL_RPATH "${SENSEGLOVE_LIB_DIR}"
)

# The SDK exposes libc++-based C++ interfaces.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(senseglove_reader PRIVATE -stdlib=libc++)
    target_link_options(senseglove_reader PRIVATE -stdlib=libc++)
endif()
