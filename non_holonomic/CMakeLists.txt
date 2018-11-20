# Holonomic Robot path planning using the Modular A* library

add_executable(non_holonomic
    non_holonomic.cpp
)

target_link_libraries(non_holonomic
    PUBLIC
        hybrid_non_holonomic_lib
    PUBLIC
        modular_a_star
)