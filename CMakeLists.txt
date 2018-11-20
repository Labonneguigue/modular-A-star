cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(modular_a_star)

add_definitions(-std=c++11)

# Base library
add_subdirectory(modular_a_star)

# Holonomic Robot Motion Model
add_subdirectory(hybrid_holonomic)
add_subdirectory(holonomic)

# Non Holonomic Robot Motion Model
add_subdirectory(non_holonomic)
add_subdirectory(hybrid_non_holonomic)