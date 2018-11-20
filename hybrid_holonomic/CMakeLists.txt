# Holonomic Robot path planning using the Modular A* library
# Hybrid refers to the use of a heuristic function to determine
# which cell to explore first: Hybrid A* Algorithm

add_executable(hybrid_holonomic
    hybrid_holonomic.cpp
)

target_link_libraries(hybrid_holonomic
    PUBLIC
        modular_a_star
)