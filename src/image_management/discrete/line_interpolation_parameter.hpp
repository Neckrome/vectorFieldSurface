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

#ifndef LINE_INTERPOLATION_PARAMETER_HPP
#define LINE_INTERPOLATION_PARAMETER_HPP

#include <iostream>
#include "template/line_basic.hpp"




namespace cpe
{
struct line_discrete;


/** \brief Container for float values. Store coefficients for interpolation. */
struct line_interpolation_parameter : line_basic<float>
{
    line_interpolation_parameter();

    /** Compute interpolation coefficient for a given line
        The coefficient are computed as the relative length for each entry of the line.

        The first entry is always 0.0
        If the size is greater than 1, the last entry is 1.0
        For other entries, the coefficient value is current_length/total_length of the line.   */
    line_interpolation_parameter(line_discrete const& line);

};


}

#endif
