#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include "modular_a_star.h"

template<typename stateT>
class Map : public IMap
{
    virtual ~Map() {};

    virtual bool isAccessible(stateT state);
};

#endif