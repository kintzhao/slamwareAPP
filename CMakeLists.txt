cmake_minimum_required(VERSION 2.8.3)
project(slamwareAPP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SDK_DIRS "${PROJECT_SOURCE_DIR}/SDK/slamware_sdk_linux-x86_64-gcc4.8")
set(SDK_DEBUG true)

if(SDK_DEBUG)
    set(SDK_INCLUDE_DIR "${SDK_DIRS}/linux-x86_64-debug")
    set(SDK_LIBRARY_DIR "${SDK_DIRS}/linux-x86_64-debug")
else()
    set(SDK_INCLUDE_DIR "${SDK_DIRS}/linux-x86_64-release")
    set(SDK_LIBRARY_DIR "${SDK_DIRS}/linux-x86_64-release")
endif()

message("${SDK_INCLUDE_DIR}")
message("${SDK_LIBRARY_DIR}")
include_directories(
   ${SDK_INCLUDE_DIR}/include
)

link_directories( ${SDK_LIBRARY_DIR}/lib)

add_executable(moveandpathdemo ./src/main.cpp)

target_link_libraries(moveandpathdemo
        librpos_robotplatforms_rpslamware.a
        librpos_framework.a
        libbase64.a
        librlelib.a
        libjsoncpp.a
        libcurl.a
        libssl.a
        libcrypto.a
        libboost_atomic.a
        libboost_chrono.a
        libboost_date_time.a
        libboost_regex.a
        libboost_system.a
        libboost_thread.a
        pthread
)

