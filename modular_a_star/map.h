#ifndef MAP_H
#define MAP_H

#include <vector>
#include "grids.h"

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

    virtual void printGrid() const = 0;

};

/**
 * 2D Map defines the behavior of the underlying physical map
 */
template<typename stateT>
class Map2D : public IMap<stateT>
{
public:

    Map2D(std::vector<std::vector<int> >& grid)
        : mGrid(grid)
    {}

    virtual ~Map2D() {};

    /** Dictates whether a location on the map is accessible or not
     *
     * @param[in] state State at potential destination location
     */
    virtual bool isAccessible(stateT state) const override
    {
        return (state.getX() >= 0 && state.getX() < mGrid.size()
             && state.getY() >= 0 && state.getY() < mGrid[0].size());
    }

    char visChar(const int state) const
    {
        switch(state)
        {
            case X: return 'X';
            case pp::visited: return '*';
            case start: return '?';
            case end: return '@';
            default:
            case pp::_: return '0';
        }
    }

    virtual void printGrid() const override
    {
        for(int i = 0; i < mGrid.size(); i++)
        {
            for(int j = 0; j < mGrid[0].size(); j++)
            {
                std::cout << " " << visChar(mGrid[i][j]);
            }
            std::cout << std::endl;
        }
    }

private:

    std::vector<std::vector<int> >& mGrid; ///< Reference to the grid representing the map itself.
};

} // END: pp

#endif