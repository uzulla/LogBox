# Host-side unit tests for the gnss component's UBX helpers and the record (CSV / JSON) formatting, using the Unity bundled with ESP-IDF:
#   cmake -S test/host -B test/host/build && cmake --build test/host/build && test/host/build/test_gnss
cmake_minimum_required(VERSION 3.22)
project(gnss_host_test C CXX)
set(CMAKE_CXX_STANDARD 20)

set(UNITY_DIR $ENV{IDF_PATH}/components/unity/unity/src)
add_executable(test_gnss
    test_gnss.cpp
    test_record.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../components/record/record.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../components/gnss/ubx.cpp
    ${UNITY_DIR}/unity.c)
target_include_directories(test_gnss PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../../components/gnss/include
    ${CMAKE_CURRENT_LIST_DIR}/../../components/imu/include
    ${CMAKE_CURRENT_LIST_DIR}/../../components/record/include ${UNITY_DIR})
target_compile_options(test_gnss PRIVATE -Wall -Wextra)
