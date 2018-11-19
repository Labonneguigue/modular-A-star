#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "hybrid_a_star.h"
#include "modular_a_star.h"
#include "map.h"

#define STEPS_DETAILS 0

int main() {

    std::vector<std::vector<int> > maze = pp::maze1;

    pp::Map2D<BicycleModelState> map(maze);
    pp::A_Star<BicycleModelState, BicycleModelState::cNumberThetaCells> AStar(map);

    BicycleModelState start(0.0, 0.0, 0.0);
    BicycleModelState end(maze.size()-1, maze[0].size()-1, 0.0);

    maze[start.getX()][start.getY()] = pp::start;
    maze[end.getX()][end.getY()] = pp::end;

    std::vector<std::vector<std::vector<BicycleModelState> > > pathToGoal;

    std::cout << "Finding path through grid:\n";
    map.printGrid();

    if( AStar.search(maze, pathToGoal, start, end) )
    {
        std::vector<BicycleModelState> path = AStar.reconstructPath(pathToGoal, start, end);

        for (auto state = path.rbegin(); state != path.rend(); ++state)
        {
#if STEPS_DETAILS
            std::cout << "### Step: " << state->getIteration() << "###\n";
            std::cout << "x: " << state->getX() << "   y: " << state->getY() << "\n";
            std::cout << "theta: " << state->getInternalState() << "\n";
#endif
            maze[state->getX()][state->getY()] = pp::visited;
        }

        maze[start.getX()][start.getY()] = pp::start;
        maze[end.getX()][end.getY()] = pp::end;

        map.printGrid();
    }

    return 0;
}