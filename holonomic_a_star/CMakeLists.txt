# Holonomic Robot path planning using the Modular A* library

add_executable(holonomic_a_star
    main.cpp
)

target_link_libraries(holonomic_a_star
    PUBLIC
        modular_a_star
)