# Hybrid A* library using the Modular A* library

add_library(hybrid_a_star INTERFACE)

target_include_directories(hybrid_a_star
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(hybrid_a_star
    INTERFACE
        modular_a_star
)