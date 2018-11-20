#ifndef STATE_H
#define STATE_H

#include <vector>

/** Path Planner namespace
 */
namespace pp {

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

    virtual void expand(std::vector<IState*>& nextStates, const IState& goal) const = 0;

    virtual bool hasReached(const IState& goal) const = 0;

    virtual bool operator<(const IState& rhs) const = 0;
};

/**
 * Class defining the state of the moving object through the maze
 * which can move in any direction independently of the previous
 * motion. Describes a holonomic robot motion.
 */
class HolonomicState : public IState
{
public:

    HolonomicState( int x = 0
                  , int y = 0
                  , int g = 0)
        : x(x)
        , y(y)
        , g(g)
    {}

    virtual ~HolonomicState() {};
    virtual int getX() const override { return x; };
    virtual int getY() const override { return y; };
    virtual int getIteration() const override { return g;};
    virtual int getHeuristic() const override { return 0;};

    /** Only 1 overlapping state per location meaning that the robot
     *  can only visit each locations on the map once.
     */
    virtual int getInternalState() const override
    { return cNumberOfStatesPerLocations-1; };

    /**
     * Creates all possible next states from the current state
     *
     * @param[in] goal Goal state used to determine the heuristic
     */
    virtual void expand(std::vector<IState*>& nextStates, const IState& goal) const override
    {
        nextStates.emplace_back(new HolonomicState(x,y+1, g+1));
        nextStates.emplace_back(new HolonomicState(x-1,y, g+1));
        nextStates.emplace_back(new HolonomicState(x+1,y, g+1));
        nextStates.emplace_back(new HolonomicState(x,y-1, g+1));
    }

    virtual bool hasReached(const IState& goal) const override
    {
        return (getX() == goal.getX() && getY() == goal.getY());
    }

    /** Comparator for sorting the state in the A* algorithm
     */
    bool operator<(const pp::IState& rhs) const override
    {
        return (g < rhs.getIteration());
    }

public:

    static constexpr int cNumberOfStatesPerLocations = 1;

protected:

    int x; ///< x coordinate of the location on the grid
    int y; ///< y coordinate of the location on the grid
    int g; ///< iteration count

};

} // END: pp

#endif