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

#ifndef LINE_BASIC_HPP
#define LINE_BASIC_HPP

#include <vector>


namespace cpe
{

/** Container for vector of generic values */
template <typename VALUE>
struct line_basic
{
    line_basic();
    line_basic(std::initializer_list<VALUE> const& param);

    /** Get the size of the coordinate vector */
    int size() const;

    /** Access to the k-th element */
    VALUE const& operator[](int index) const;
    /** Access to the k-th element */
    VALUE const& operator()(int index) const;

    /** STL compatibility ranged-for loop */
    typename std::vector<VALUE>::const_iterator begin() const;
    /** STL compatibility ranged-for loop */
    typename std::vector<VALUE>::const_iterator end() const;
    /** STL compatibility ranged-for loop */
    typename std::vector<VALUE>::const_iterator cbegin() const;
    /** STL compatibility ranged-for loop */
    typename std::vector<VALUE>::const_iterator cend() const;

protected:

    /** Storage for the coordinates */
    std::vector<VALUE> data;
};

/** Standard output */
template <typename VALUE>
std::ostream& operator<<(std::ostream& stream,line_basic<VALUE> const& line);


}

#include "line_basic.tpp"

#endif
