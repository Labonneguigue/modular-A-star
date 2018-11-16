#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "hybrid_a_star.h"

void printGrid(const std::vector<std::vector<int> >& grid)
{
    for(int i = 0; i < grid.size(); i++)
    {
        std::cout << grid[i][0];
        for(int j = 1; j < grid[0].size(); j++)
        {
            std::cout << "," << grid[i][j];
        }
        std::cout << std::endl;
    }
}

int main() {

    int X = 1;
    int _ = 0;

    std::vector<std::vector<int> > maze1 = {
        {_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,},
        {_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,},
        {_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,},
        {_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_,},
        {_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_,},
        {_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_,},
        {_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_,},
        {_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_,},
        {_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,},
        {_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X,},
        {_,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X,},
        {_,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,},
        {_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,},
        {_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,},
        {_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,},
        {X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,},
    };

    std::vector<std::vector<int> > maze2 = {
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
        {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
    };

    std::vector<std::vector<int> > maze3 = {
        {_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_},
        {_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,X,X,X,_,_,_,_,_,_,_,X,_,_,_,_,_,_},
        {_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,X,X,X,_,_,_,_,_},
        {X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,X,X,X,X,X,_,_,_,_},
    };

    std::vector<std::vector<int> > maze = maze3;

    Map<MazeState> map(maze);
    pp::A_Star<MazeState, MazeState::cNumberThetaCells> AStar(map);

    MazeState start(0.0, 0.0, 0.0);
    MazeState end(maze.size()-1, maze[0].size()-1, 0.0);

    std::vector<std::vector<std::vector<MazeState> > > pathToGoal;

    std::cout << "Finding path through grid:\n";
    printGrid(maze);

    if( AStar.search(maze, pathToGoal, start, end) )
    {
        std::vector<MazeState> path = AStar.reconstructPath(pathToGoal, start, end);

        for (auto state = path.rbegin(); state != path.rend(); ++state)
        {
            std::cout << "### Step: " << state->getIteration() << "###\n";
            std::cout << "x: " << state->getX() << "   y: " << state->getY() << "\n";
            std::cout << "theta: " << state->getInternalState() << "\n";
            maze[state->getX()][state->getY()] = 3;
        }

        printGrid(maze);
    }

    return 0;
}