#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "hybrid_a_star.h"
#include "modular_a_star.h"
#include "map.h"


int main() {

    std::vector<std::vector<int> > maze = pp::maze3;

    pp::Map2D<BicycleModelState> map(maze);
    pp::A_Star<BicycleModelState, BicycleModelState::cNumberThetaCells> AStar(map);

    BicycleModelState start(0.0, 0.0, 0.0);
    BicycleModelState end(maze.size()-1, maze[0].size()-1, 0.0);

    std::vector<std::vector<std::vector<BicycleModelState> > > pathToGoal;

    std::cout << "Finding path through grid:\n";
    map.printGrid();

    if( AStar.search(maze, pathToGoal, start, end) )
    {
        std::vector<BicycleModelState> path = AStar.reconstructPath(pathToGoal, start, end);

        for (auto state = path.rbegin(); state != path.rend(); ++state)
        {
            std::cout << "### Step: " << state->getIteration() << "###\n";
            std::cout << "x: " << state->getX() << "   y: " << state->getY() << "\n";
            std::cout << "theta: " << state->getInternalState() << "\n";
            maze[state->getX()][state->getY()] = 3;
        }

        map.printGrid();
    }

    return 0;
}