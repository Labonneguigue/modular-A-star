# Non Holonomic Hybrid A* library using the Modular A* library

add_library(hybrid_non_holonomic_lib INTERFACE)

target_include_directories(hybrid_non_holonomic_lib
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(hybrid_non_holonomic_lib
    INTERFACE
        modular_a_star
)

# Executable

add_executable(hybrid_non_holonomic
    "${CMAKE_CURRENT_SOURCE_DIR}/hybrid_non_holonomic.cpp"
)

target_link_libraries(hybrid_non_holonomic
    PUBLIC
        hybrid_non_holonomic_lib
)