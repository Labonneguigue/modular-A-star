#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "modular_a_star.h"
#include "state.h"
#include "map.h"

int main() {

    std::vector<std::vector<int> > maze = pp::maze4;

    pp::Map2D<pp::HolonomicState> map(maze);
    pp::A_Star<pp::HolonomicState, pp::HolonomicState::cNumberOfStatesPerLocations> AStar(map);

    pp::HolonomicState start(0, 0, 0);
    pp::HolonomicState end(maze.size()-1, maze[0].size()-1, 0);

    std::vector<std::vector<std::vector<pp::HolonomicState> > > pathToGoal;

    std::cout << "Finding path through grid:\n";
    map.printGrid();

    AStar.search(maze, pathToGoal, start, end);

    std::vector<pp::HolonomicState> path = AStar.reconstructPath(pathToGoal, start, end);

    for (auto state = path.rbegin(); state != path.rend(); ++state)
    {
        /*std::cout << "### Step: " << state->getIteration() << "###\n";
        std::cout << "x: " << state->getX() << "   y: " << state->getY() << "\n";
        std::cout << "theta: " << state->getInternalState() << "\n";*/
        maze[state->getX()][state->getY()] = pp::visited;
    }

    maze[start.getX()][start.getY()] = pp::start;
    maze[end.getX()][end.getY()] = pp::end;

    map.printGrid();

    return 0;
}