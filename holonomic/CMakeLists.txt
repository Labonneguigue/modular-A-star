# Holonomic Robot path planning using the Modular A* library

add_executable(holonomic
    holonomic.cpp
)

target_link_libraries(holonomic
    PUBLIC
        modular_a_star
)