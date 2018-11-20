#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "modular_a_star.h"
#include "state.h"
#include "map.h"

#define STEPS_DETAILS 0

/** Path Planner namespace
 */
namespace pp {

class HolonomicHybridState : public HolonomicState
{
public:

    //using pp::HolonomicHybridState::x;

    HolonomicHybridState( const int x = 0
                        , const int y = 0
                        , const int g = 0
                        , const int f = 0)
        : HolonomicState(x, y, g)
        , f(f)
    {}

    virtual ~HolonomicHybridState() {};

    virtual int getHeuristic() const override { return f; };

    virtual void expand(std::vector<IState*>& nextStates, const IState& goal) const override
    {
        nextStates.emplace_back(new HolonomicHybridState(x, y+1, g+1
                                             , heuristic(x, y+1, goal)));
        nextStates.emplace_back(new HolonomicHybridState(x-1, y, g+1
                                             , heuristic(x-1, y, goal)));
        nextStates.emplace_back(new HolonomicHybridState(x+1, y, g+1
                                             , heuristic(x+1, y, goal)));
        nextStates.emplace_back(new HolonomicHybridState(x, y-1, g+1
                                             , heuristic(x, y-1, goal)));
    }

private:

    int heuristic(const int x, const int y, const IState& goal) const
    {
        return (std::abs(goal.getX()-x) + std::abs(goal.getY()-y));
        std::cout << "h\n";
    }

    int f; ///< heuristic, euclidean distance to the goal
};

} // END: pp


int main() {

    std::vector<std::vector<int> > maze = pp::maze4;

    pp::Map2D<pp::HolonomicHybridState> map(maze);
    pp::A_Star<pp::HolonomicHybridState, pp::HolonomicState::cNumberOfStatesPerLocations> AStar(map);

    pp::HolonomicHybridState start(0, 0, 0);
    pp::HolonomicHybridState end(maze.size()-1, maze[0].size()-1, 0);

    maze[start.getX()][start.getY()] = pp::start;
    maze[end.getX()][end.getY()] = pp::end;

    std::vector<std::vector<std::vector<pp::HolonomicHybridState> > > pathToGoal;

    std::cout << "Finding path through grid:\n";
    map.printGrid();

    AStar.search(maze, pathToGoal, start, end);

    std::vector<pp::HolonomicHybridState> path = AStar.reconstructPath(pathToGoal, start, end);

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