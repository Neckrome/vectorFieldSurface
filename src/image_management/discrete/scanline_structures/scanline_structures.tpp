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
#ifndef SCANLINE_STRUCTURES_TPP
#define SCANLINE_STRUCTURES_TPP

#include "scanline_structures.hpp"

namespace cpe
{

template <typename T>
std::ostream& operator<<(std::ostream& stream,position_info<T> const& info)
{
    stream<<info.coordinate<<" , "<<info.parameter;
    return stream;
}

template <typename T>
position_left_right<T>::position_left_right()
    :left({{-1,-1},T()}),right({{-1,-1},T()})
{}

template <typename T>
position_left_right<T>::position_left_right(position_info<T> const& param)
    :left(param),right(param)
{}

template <typename PARAMETER>
std::ostream& operator<<(std::ostream& stream,position_left_right<PARAMETER> const& param)
{
    stream<<"["<<param.left<<"] - ["<<param.right<<"]";
    return stream;
}

template <typename T>
void position_left_right<T>::update(position_info<T> const& param)
{
    int const x_candidate = param.coordinate.x();
    int const x_left_current = left.coordinate.x();
    int const x_right_current = right.coordinate.x();

    if(x_left_current > x_candidate)
        left = param;
    else if(x_right_current < x_candidate)
        right = param;
}





}

#endif
