cmake_minimum_required(VERSION 3.5)
project(test_dda)

set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")


add_executable(test_dda test/test.cpp)