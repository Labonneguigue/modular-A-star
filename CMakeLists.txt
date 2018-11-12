cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(modular_a_star)

add_subdirectory(src)


# Executable instantiating the Hybrid A* library

add_executable(hybrid_a_star_example
    "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
)

target_link_libraries(hybrid_a_star_example
    hybrid_a_star
)