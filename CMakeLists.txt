cmake_minimum_required(VERSION 3.10.0)
project(beagle-bot VERSION 0.1.0 LANGUAGES C CXX)

add_executable(beagle-bot main.cpp)
target_include_directories(beagle-bot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

include(CTest)
enable_testing()

