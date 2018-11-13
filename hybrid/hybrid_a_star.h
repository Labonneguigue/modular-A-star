#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include <cmath>
#include "modular_a_star.h"

template<typename stateT>
class Map : public pp::IMap<stateT>
{
public:

    virtual ~Map() {};

    virtual bool isAccessible(stateT state) const override
    {

        return true;
    }

    virtual bool hasReached(stateT current, stateT goal) const override
    {
        return true;
    }
};

class MazeState : public pp::IState
{
public:

    virtual int getX() const override { return x;};
    virtual int getY() const override { return x;};
    virtual int getIteration() const override { return g;};
    virtual int getHeuristic() const override { return f;};

    /*
     *   Takes an angle (in radians) and returns which "stack" in the 3D configuration space
     *   this angle corresponds to. Angles near 0 go in the lower stacks while angles near
     *   2 * pi go in the higher stacks.
     */
    virtual int getInternalState() const override
    {
        double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
        int stack_number = (int)(round(new_theta * cNumberThetaCells / (2*M_PI))) % cNumberThetaCells;
        return stack_number;
    }

    virtual std::vector<IState> expand(IState& goal) const override
    {
        std::vector<IState> nextStates;

        return nextStates;
    }

    /** Comparator for sorting the state in the A* algorithm
     */
    bool operator<(const pp::IState& rhs) const override
    {
        return (g + f) < (rhs.getIteration() + rhs.getHeuristic());
    }

public:

    static constexpr int cNumberThetaCells = 90;

private:

    static constexpr double cSpeed = 1.45;

    int g; // iteration
    int f; // heuristic
    int x;
    int y;
    double theta;

};

#endif