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

#ifndef LINE_DISCRETE_HPP
#define LINE_DISCRETE_HPP

#include <vector>
#include "ivec2.hpp"
#include "template/line_basic.hpp"

namespace cpe
{

/** Container for vector of 2D integer coordinates */
struct line_discrete : line_basic<ivec2>
{
    using line_basic<ivec2>::line_basic;

    /** Add a new coordinate in the vector */
    void push_back(ivec2 const& position);
};


}

#endif
