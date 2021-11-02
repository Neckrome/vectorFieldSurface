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

#ifndef LINE_BASIC_TPP
#define LINE_BASIC_TPP

#include "line_basic.hpp"

#include "../../lib/common/error_handling.hpp"

namespace cpe
{

template <typename VALUE>
line_basic<VALUE>::line_basic()
    :data()
{}

template <typename VALUE>
line_basic<VALUE>::line_basic(std::initializer_list<VALUE> const& param)
    :data(param)
{}

template <typename VALUE>
int line_basic<VALUE>::size() const
{
    return data.size();
}

template <typename VALUE>
VALUE const& line_basic<VALUE>::operator[](int index) const
{
    ASSERT_CPE(index>=0 && index<size(),"index outside allowed space");
    return data[index];
}

template <typename VALUE>
VALUE const& line_basic<VALUE>::operator()(int index) const
{
    return (*this)[index];
}

template <typename VALUE>
typename std::vector<VALUE>::const_iterator line_basic<VALUE>::begin() const  { return data.cbegin(); }
template <typename VALUE>
typename std::vector<VALUE>::const_iterator line_basic<VALUE>::end() const    { return data.cend(); }
template <typename VALUE>
typename std::vector<VALUE>::const_iterator line_basic<VALUE>::cbegin() const { return data.cbegin(); }
template <typename VALUE>
typename std::vector<VALUE>::const_iterator line_basic<VALUE>::cend() const   { return data.cend(); }


template <typename VALUE>
std::ostream& operator<<(std::ostream& stream,line_basic<VALUE> const& line)
{
    for(VALUE const& p: line)
        stream<<p<<" ";
    return stream;
}


}

#endif
