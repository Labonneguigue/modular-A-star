# Modular A* as a standalone library

add_library(modular_a_star
    "${CMAKE_CURRENT_SOURCE_DIR}/modular_a_star.cpp"
)

target_include_directories(modular_a_star
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/modular_a_star.h>
)

# Hybrid A* library using the Modular A* library

add_library(hybrid_a_star
    "${CMAKE_CURRENT_SOURCE_DIR}/hybrid_a_star.cpp"
)

target_include_directories(hybrid_a_star
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hybrid_a_star.h
)

target_link_libraries(hybrid_a_star
    modular_a_star
)