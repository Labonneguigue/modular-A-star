cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(modular_a_star)

add_definitions(-std=c++11)

add_subdirectory(hybrid)
add_subdirectory(modular_a_star)
add_subdirectory(holonomic_a_star)

# Executable instantiating the Hybrid A* library

add_executable(hybrid_a_star_example
    main.cpp
)

target_link_libraries(hybrid_a_star_example
    hybrid_a_star
)