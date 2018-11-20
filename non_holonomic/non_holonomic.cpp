#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "state.h"
#include "map.h"
#include "modular_a_star.h"
#include "bicyclemodelstate.h"

#define STEPS_DETAILS 0

/** Same class as the BicycleModelState but without the heuristic
 */
class NoHeuristicBicycleModelState : public BicycleModelState
{
public:

    using BicycleModelState::cMaximumSteering;
    using BicycleModelState::cDeltaSteering;
    using BicycleModelState::cSpeed;
    using BicycleModelState::cVehicleLength;

    NoHeuristicBicycleModelState(const double x = 0.0
                     , const double y = 0.0
                     , const double theta = 0.0
                     , const int g = 0
                     , const int f = 0)
    : BicycleModelState(x, y, theta, g, f)
    {}

    /**
     * Creates all possible next states from the current state
     *
     * @param[in] goal Goal state used to determine the heuristic
     */
    virtual void expand(std::vector<IState*>& nextStates, const IState& goal) const
    {
        const int newG = g+1;

        for (double delta = -cMaximumSteering; delta < cMaximumSteering+1.0 ; delta+=cDeltaSteering)
        {
            const double deltaRad = utl::deg2rad(delta);
            const double omega = cSpeed / cVehicleLength * tan(deltaRad);
            double newTheta = theta + omega;
            newTheta = utl::warp2Pi(newTheta);
            const double newX = x + cSpeed * cos(theta);
            const double newY= y + cSpeed * sin(theta);

            nextStates.emplace_back(new NoHeuristicBicycleModelState(newX, newY, newTheta, newG));
        }
    }

};


int main() {

    std::vector<std::vector<int> > maze = pp::maze4;

    pp::Map2D<NoHeuristicBicycleModelState> map(maze);
    pp::A_Star<NoHeuristicBicycleModelState
              ,NoHeuristicBicycleModelState::cNumberThetaCells> AStar(map);

    NoHeuristicBicycleModelState start(0.0, 0.0, 0.0);
    NoHeuristicBicycleModelState end(maze.size()-1, maze[0].size()-1, 0);

    maze[start.getX()][start.getY()] = pp::start;
    maze[end.getX()][end.getY()] = pp::end;

    std::vector<std::vector<std::vector<NoHeuristicBicycleModelState> > > pathToGoal;

    std::cout << "Finding path through grid:\n";
    map.printGrid();

    AStar.search(maze, pathToGoal, start, end);

    std::vector<NoHeuristicBicycleModelState> path = AStar.reconstructPath(pathToGoal, start, end);

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

    return 0;
}