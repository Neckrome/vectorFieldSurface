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

#ifndef SCANLINE_STRUCTURES_HPP
#define SCANLINE_STRUCTURES_HPP

#include "../ivec2.hpp"



namespace cpe
{

/** Structure storing a directe coordinate and a generic parameter.
   Will be used as the min/max value for the scanline container  */
template <typename PARAMETER>
struct position_info
{
    ivec2 coordinate;     // The coordinate associated with the parameter
    PARAMETER parameter;  // A generic parameter (color, depth, texture coordinate, ...)
};

/** Standard output for a position_info */
template <typename PARAMETER>
std::ostream& operator<<(std::ostream& stream,position_info<PARAMETER> const& info);

/** Storage for a given line of the scanline.
 *  Store the most-left/most-right value called respectively min and max. */
template <typename PARAMETER>
struct position_left_right
{
    position_left_right();
    position_left_right(position_info<PARAMETER> const& param);

    /** Update an existing value (set the left/right value depending on the x-coordinate) */
    void update(position_info<PARAMETER> const& param);

    position_info<PARAMETER> left;
    position_info<PARAMETER> right;
};

/** Standard output for a position_left_right */
template <typename PARAMETER>
std::ostream& operator<<(std::ostream& stream,position_left_right<PARAMETER> const& param);

}



#include "scanline_structures.tpp"

#endif
