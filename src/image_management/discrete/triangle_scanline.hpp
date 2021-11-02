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


#pragma once

#ifndef TRIANGLE_SCANLINE_HPP
#define TRIANGLE_SCANLINE_HPP

#include "scanline_structures/scanline_structures.hpp"
#include "ivec2.hpp"

#include <map>


namespace cpe
{

template <typename T>
using triangle_scanline_iterator = typename std::map<int,position_left_right<T> >::const_iterator ;

/** Container and algorithm for scanline data over a triangle.
    Given 3 discrete position of a triangle, and the associated 3 parameters,
     compute for each y values spanned by the triangle the most left and right x-position, and
     the linearly interpolated parameter.
    The structure assume that the parameter can be interpolated using standard linear algebra operators. */
template <typename PARAMETER>
struct triangle_scanline
{
    /** Construct the scanline data given 3 discrete positions and 3 parameters */
    triangle_scanline(ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,
                      PARAMETER const& param_0,PARAMETER const& param_1,PARAMETER const& param_2);

    /** STL compatible ranged loop */
    triangle_scanline_iterator<PARAMETER> begin() const;
    /** STL compatible ranged loop */
    triangle_scanline_iterator<PARAMETER> end() const;
    /** STL compatible ranged loop */
    triangle_scanline_iterator<PARAMETER> cbegin() const;
    /** STL compatible ranged loop */
    triangle_scanline_iterator<PARAMETER> cend() const;

private:

    /** The internal data stored as a map of position_left_right */
    std::map<int,position_left_right<PARAMETER> > data;
};

/** Standard output for triangle_scanline */
template <typename PARAMETER>
std::ostream& operator<<(std::ostream& stream,triangle_scanline<PARAMETER> const& scanline);

/** Factory for triangle_scanline.
 * Enables automatic template deduction for the parameter.  */
template <typename PARAMETER>
triangle_scanline<PARAMETER> triangle_scanline_factory(ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,
                                                       PARAMETER const& param_0,PARAMETER const& param_1,PARAMETER const& param_2);

}



#include "triangle_scanline.tpp"

#endif
