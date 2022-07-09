
cmake_minimum_required(VERSION 3.11.3)

project(KALMAN)

set(PROJECT_NAME KALMAN)
set(TEST_NAME KALMAN_TEST)

# Set the C++ standard we will use
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)

add_subdirectory(test)

add_dependencies(${TEST_NAME} ${PROJECT_NAME})