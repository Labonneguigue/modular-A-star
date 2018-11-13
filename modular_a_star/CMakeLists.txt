# Modular A* as a standalone library

add_library(modular_a_star INTERFACE)

target_include_directories(modular_a_star
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)