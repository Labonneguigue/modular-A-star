#ifndef MODULAR_A_STAR_H
#define MODULAR_A_STAR_H

/** Path Planner namespace
 */
namespace pp {

template<typename stateT>
class IMap
{
    virtual ~IMap() {};

    virtual bool isAccessible(stateT state) = 0;

    virtual bool hasReached(stateT state, stateT goal) = 0;
};


template<typename stateT, int states>
class A_Star
{
public:
    /** A* Contrustor takes as input a reference to a state
     */
    A_Star(const IMap& map);

    /** Search for path from start to goal.
     * Expands the state provided as class template argument at each step.
     */
    search( std::vector<std::vector<int>>& grid
          , stateT start
          , stateT goal )
    {
        // Tri dimensional vector for storing all spacial positions on the grid
        // as well as the states within each positions that have been visited and closed.
        std::vector<std::vector<std::vector<int>>> closed( states , std::vector<std::vector<int>>(grid[0].size()
                                                                  , std::vector<int>(grid.size())));

        // Tri dimensional vector for storing all states to be expanded.
        std::vector<std::vector<std::vector<int>>> previous( states , std::vector<std::vector<stateT>>(grid[0].size()
                                                                    , std::vector<stateT>(grid.size())));

    }

    /** Uses the 3D vector of past locations and states to reconstruct the path
     *
     * @return Path Sequence of states from the end to the beginning.
     */
    std::vector<stateT> reconstructPath(std::vector<std::vector<std::vector<stateT>>> from, stateT start, stateT end)
    {
        std::vector<stateT> path = { end };
        int internalState = end.getInternalState();
        stateT current = from[internalState][final.getX()][final.getY()];
        internalState = current.getInternalState();

        while (IMap.hasReached(current, start))
        {
            path.push_back(current);
            current = from[internalState][current.getX()][current.getY()];
            internalState = current.getInternalState();
        }

        return path;
    }

private:

    const IMap& mMap; ///< Map object able to tell wheather the state is valid or not and signal when the goal has been reached.

};

} // END: pp

#endif