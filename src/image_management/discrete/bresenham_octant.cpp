/*
**    TP CPE Lyon
**    Copyright (C) 2015 Damien Rohmer
**
**    This program is free software: you can redistribute it and/or modify
**    it under the terms of the GNU General Public License as published by
**    the Free Software Foundation, either version 3 of the License, or
**    (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**    but WITHOUT ANY WARRANTY; without even the implied warranty of
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**    GNU General Public License for more details.
**
**    You should have received a copy of the GNU General Public License
**    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "bresenham_octant.hpp"

#include "../lib/common/error_handling.hpp"

#include "ivec2.hpp"

namespace cpe
{


int octant_number(ivec2 const& p0,ivec2 const& p1)
{
    int const x0 = p0.x();
    int const y0 = p0.y();

    int const x1 = p1.x();
    int const y1 = p1.y();

    int const dx = x1-x0;
    int const dy = y1-y0;

    if( dy>=0 )
    {
        if( dx>=0 )
        {
            if( dx>=dy )   return 0;
            else           return 1;
        }
        else
        {
            if( -dx>=dy )  return 3;
            else           return 2;
        }
    }
    else
    {
        if( dx>=0 )
        {
            if( dx>=-dy )  return 7;
            else           return 6;
        }
        else
        {
            if( -dx>=-dy ) return 4;
            else           return 5;
        }
    }

    throw cpe::exception_cpe("Critical error",EXCEPTION_PARAMETERS_CPE);
}

// Going from original position to first octant
ivec2 symmetry_octant(ivec2 const& p,int const octant)
{
    ASSERT_CPE(octant>=0 && octant<8,"Octant number should be in [0,7]");

    int const x = p.x();
    int const y = p.y();

    switch(octant)
    {
    case 0: return p;
    case 1: return ivec2 ( y, x);
    case 2: return ivec2 ( y,-x);
    case 3: return ivec2 (-x, y);
    case 4: return ivec2 (-x,-y);
    case 5: return ivec2 (-y,-x);
    case 6: return ivec2 (-y, x);
    case 7: return ivec2 ( x,-y);
    default:
        throw cpe::exception_cpe("critical error",EXCEPTION_PARAMETERS_CPE);
    }
}

// Going from first octant to original position
ivec2 symmetry_octant_inverse(ivec2 const& p,int const octant)
{
    ASSERT_CPE(octant>=0 && octant<8,"Octant number should be in [0,7]");

    int const x = p.x();
    int const y = p.y();

    switch(octant)
    {
    case 0: return p;
    case 1: return ivec2 ( y, x);
    case 2: return ivec2 (-y, x);
    case 3: return ivec2 (-x, y);
    case 4: return ivec2 (-x,-y);
    case 5: return ivec2 (-y,-x);
    case 6: return ivec2 ( y,-x);
    case 7: return ivec2 ( x,-y);
    default:
        throw cpe::exception_cpe("critical error",EXCEPTION_PARAMETERS_CPE);
    }
}


}
