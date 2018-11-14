#ifndef MODULAR_A_STAR_H
#define MODULAR_A_STAR_H

#include <iostream>
#include <vector>

/** Path Planner namespace
 */
namespace pp {

/** Interface which defines how the map provided the the A* solver
 * should behave to find a solution
 */
template<typename stateT>
class IMap
{
public:

    virtual ~IMap() {};

    virtual bool isAccessible(stateT state) const = 0;

};

/** Interface defining what a state type should implement to be
 * acceptable for the A star algorithm
 */
class IState
{
public:

    virtual ~IState() {};

    virtual int getX() const = 0;

    virtual int getY() const = 0;

    virtual int getIteration() const = 0;

    virtual int getHeuristic() const = 0;

    virtual int getInternalState() const = 0;

    //virtual std::vector<IState> expand(const IState& goal) const = 0;

    virtual bool hasReached(const IState& goal) const = 0;

    virtual bool operator<(const IState& rhs) const = 0;
};

/** A Star algorithm - Path planner solver
 */
template<typename stateT, int states>
class A_Star
{
public:

    /** A* Contrustor takes as input a reference to a state
     */
    A_Star(const IMap<stateT>& map)
        : mMap(map)
    {}

    /** Search for path from start to goal.
     * Expands the state provided as class template argument at each step.
     *
     * @param[in] grid Grid map with all possible and impossible locations
     * @param[out] pathToGoal Path taken from beginning to the end
     * @param[in] start Starting state
     * @param[in|out] goal Goal to reach as inputs comes out as the reach state if success.
     *
     * @return Bool if final state has been reached, False otherwise
     */
    bool search( std::vector<std::vector<int> >& grid
               , std::vector<std::vector<std::vector<stateT> > >& pathToGoal
               , stateT start
               , stateT& goal )
    {
        // Tri dimensional vector for storing all spacial positions on the grid
        // as well as the states within each positions that have been visited and closed.
        mClosed = std::vector<std::vector<std::vector<int> > >( states , std::vector<std::vector<int> >(grid[0].size()
                                                                     , std::vector<int>(grid.size())));

        // Tri dimensional vector for storing all states to be expanded.
        pathToGoal = std::vector<std::vector<std::vector<stateT> > >( states , std::vector<std::vector<stateT> >(grid[0].size()
                                                                           , std::vector<stateT>(grid.size())));

        // Set the first state as visited
        mClosed[start.getInternalState()][start.getX()][start.getY()] = 1;
        pathToGoal[start.getInternalState()][start.getX()][start.getY()] = start;

        // Count number of expansions
        int closedCount = 1;

        // Stack all states to be visited
        std::vector<stateT> opened = { start };

        stateT currentState;

        while(!opened.empty())
        {
            // Sort possible next steps to always expand towards the closest to the goal
            std::sort(opened.begin(), opened.end());
            currentState = opened[0];
            opened.erase(opened.begin());

            if (currentState.hasReached(goal))
            {
                std::cout << "Found path to goal in " << closedCount << " expansions.\n";
                // Return the final state by replacing the goal input argument
                goal = currentState;
                return true;
            }

            // Expand state to get all next possible states from current one
            std::vector<stateT> nextStates = currentState.expand(goal);

            for(auto& next : nextStates)
            {
                // If next state is not accessible, it is not added to the next possible states to visit
                if (mMap.isAccessible(next)) continue;

                if ( !mClosed[next.getInternalState()][next.getX()][next.getY()]
                     && !grid[next.getX()][next.getY()])
                {
                    opened.push_back(next);
                    ++mClosed[next.getInternalState()][next.getX()][next.getY()];
                    pathToGoal[next.getInternalState()][next.getX()][next.getY()] = next;
                    ++closedCount;
                }
            }
        }

        // If we end up here it's because no path was found.
        std::cout << "No valid path found.\n";
        goal = currentState;
        return false;
    }

    /** Uses the 3D vector of past locations and states to reconstruct the path
     *
     * @return Path Sequence of states from the end to the beginning.
     */
    std::vector<stateT> reconstructPath(std::vector<std::vector<std::vector<stateT> > > from, stateT start, stateT end)
    {
        std::vector<stateT> path = { end };
        int internalState = end.getInternalState();
        stateT current = from[internalState][end.getX()][end.getY()];
        internalState = current.getInternalState();

        while (!current.hasReached(start))
        {
            path.push_back(current);
            current = from[internalState][current.getX()][current.getY()];
            internalState = current.getInternalState();
        }

        return path;
    }

    /**
     * Get function to avoid overloading the search method with parameters.
     */
    void getClosedStates(std::vector<std::vector<std::vector<int> > >& closed)
    {
        closed = mClosed;
    }

private:

    const IMap<stateT>& mMap; ///< Map object able to tell wheather the state is valid or not and signal when the goal has been reached.

    std::vector<std::vector<std::vector<int> > > mClosed;

};

} // END: pp

#endif




// int g = state.g;
//   double x = state.x;
//   double y = state.y;
//   double theta = state.theta;

//   int g2 = g+1;
//   vector<HBF::maze_s> next_states;
//   for(double delta_i = -35; delta_i < 40; delta_i+=5)
//   {
//     double delta = M_PI / 180.0 * delta_i;
//     double omega = SPEED / LENGTH * tan(delta);
//     double theta2 = theta + omega;
//     if(theta2 < 0)
//     {
//         theta2 += 2*M_PI;
//     }
//     double x2 = x + SPEED * cos(theta);
//     double y2 = y + SPEED * sin(theta);
//     HBF::maze_s state2;
//     state2.g = g2;
//     state2.f = heuristic(idx(x2), idx(y2), goal);
//     state2.x = x2;
//     state2.y = y2;
//     state2.theta = theta2;
//     next_states.push_back(state2);

//   }