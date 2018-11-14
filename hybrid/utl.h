#ifndef UTL_H
#define UTL_H

#include "math.h"

namespace utl {

template<typename T>
T pi()
{
    return static_cast<T>(M_PI);
}

template<typename T>
T twoPi()
{
    return static_cast<T>(2 * M_PI);
}

template<typename T>
T deg2rad(T deg)
{
    return static_cast<T>(pi<T>() / 180.0 * deg);
}

template<typename T>
T warp2Pi(T rad)
{
    while(rad > twoPi<T>()) { rad -= twoPi<T>(); }
    while(rad < 0) { rad += twoPi<T>(); }
    return rad;
}

} // END: utl

#endif
