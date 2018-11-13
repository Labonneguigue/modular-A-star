#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

//#include "modular_a_star.h"
#include "hybrid_a_star.h"


int main() {

    MazeState mazeState;
    Map<MazeState> map;
    pp::A_Star<MazeState, MazeState::cNumberThetaCells> AStar(map);

    std::cout << "Finding path through grid:\n";

    return 0;

}