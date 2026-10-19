cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Testbench)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core core_build)

target_sources(app PRIVATE src/testbench.cpp)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../core)

# Otherwise has trouble getting core since it's up a level on the directory
target_link_libraries(app PRIVATE core)