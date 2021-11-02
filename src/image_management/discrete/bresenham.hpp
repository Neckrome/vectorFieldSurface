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

#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP


namespace cpe
{

struct line_discrete;
class ivec2;

/** Compute Bresenham algorithm between two position (p0_x,p0_y) and (p1_x,p1_y)
    \return A line_discrete structure storing all the discrete positions of the line */
void bresenham(ivec2 const& p0,ivec2 const& p1,line_discrete& line);



}

#endif
