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

/*
//
// Octant Configuration
// ---------------------
//
// We only use Bresenham algorithm in the first octant.
// For all other octants, we use symmetry criteria to go
//   back to the first octant.
//
//
//      X
//    --->
//   |
//   |
// Y v
//
//
//  Octants numbers
//
//    \5 | 6/
//   4 \ | /  7
//      \|/
//  -----------
//      /|\
//   3 / | \  0
//    /2 | 1\
//
//
*/


namespace cpe
{
class ivec2;


/** Compute the current octant of the segment [p0,p1] */
int octant_number(ivec2 const& p0,ivec2 const& p1);

/** Compute the position corresponding to the symetrical first octant */
ivec2 symmetry_octant(ivec2 const& p,int octant);

/** Compute the position corresponding the passage from the first octant to the original one */
ivec2 symmetry_octant_inverse(ivec2 const& p,int octant);

}
