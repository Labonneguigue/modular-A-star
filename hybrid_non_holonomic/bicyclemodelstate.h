#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include <cmath>

#include "state.h"
#include "utl.h"

/**
 * Class defining the state of the moving object through the maze.
 */
class BicycleModelState : public pp::IState
{
public:

    BicycleModelState( const double x = 0.0
                     , const double y = 0.0
                     , const double theta = 0.0
                     , const int g = 0
                     , const int f = 0)
        : x(x)
        , y(y)
        , theta(theta)
        , g(g)
        , f(f)
    {}

    int idx(const double position) const { return static_cast<int>(floor(position)); };

    virtual int getX() const override { return idx(x);};
    virtual int getY() const override { return idx(y);};
    virtual int getIteration() const override { return g;};
    virtual int getHeuristic() const override { return f;};

    /*
     *   Takes an angle (in radians) and returns which "stack" in the 3D configuration space
     *   this angle corresponds to. Angles near 0 go in the lower stacks while angles near
     *   2 * pi go in the higher stacks.
     */
    virtual int getInternalState() const override
    {
        double new_theta = fmod((theta + utl::twoPi<double>()),(utl::twoPi<double>()));
        int stack_number = static_cast<int>(round(new_theta * cNumberThetaCells / (utl::twoPi<double>()))) % cNumberThetaCells;
        return stack_number;
    }

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
            const int newF = heuristic(newX, newY, goal);

            nextStates.emplace_back(new BicycleModelState(newX, newY, newTheta, newG, newF));
        }
    }

    virtual bool hasReached(const IState& goal) const override
    {
        return (getX() == goal.getX() && getY() == goal.getY());
    }

    /** Comparator for sorting the state in the A* algorithm
     */
    bool operator<(const pp::IState& rhs) const override
    {
        return (g + f) < (rhs.getIteration() + rhs.getHeuristic());
    }

private:

    int heuristic(const int x, const int y, const IState& goal) const
    {
        return (std::abs(goal.getX()-x) + std::abs(goal.getY()-y));
    }

public:

    static constexpr int cNumberThetaCells = 90;

protected:

    static constexpr double cMaximumSteering = 35.0;
    static constexpr double cDeltaSteering = 5.0;
    static constexpr double cSpeed = 1.4;
    static constexpr double cVehicleLength = 0.5;

    double x;
    double y;
    double theta;

    int g; // iteration
    int f; // heuristic

};

#endif