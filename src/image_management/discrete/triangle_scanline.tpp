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
#ifndef TRIANGLE_SCANLINE_TPP
#define TRIANGLE_SCANLINE_TPP

#include "triangle_scanline.hpp"
#include "line_discrete.hpp"
#include "line_interpolation_parameter.hpp"
#include "bresenham.hpp"

namespace cpe
{


// select the first vertex of the triangle for the line given by the index
//
//     [2]
//   a----c
//    \   |
//     \  | [1]
//  [0] \ |
//       \|
//        b
//
template <typename T>
static T const& select_first_parameter(T const& a,T const& b,T const& c,int index)
{
    ASSERT_CPE(index>=0 && index<3 , "Index should be in [0,2] ");

    switch(index)
    {
    case 0: return a;
    case 1: return b;
    case 2: return c;
    }

    throw exception_cpe("Critital error",EXCEPTION_PARAMETERS_CPE);
}

// select the first vertex of the triangle for the line given by the index
template <typename T>
static T const& select_second_parameter(T const& a,T const& b,T const& c,int index)
{
    ASSERT_CPE(index>=0 && index<3 , "Index should be in [0,2] ");

    switch(index)
    {
    case 0: return b;
    case 1: return c;
    case 2: return a;
    }

    throw exception_cpe("Critital error",EXCEPTION_PARAMETERS_CPE);
}



template <typename T>
triangle_scanline<T>::triangle_scanline(ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,
                                        T const& param_0,T const& param_1,T const& param_2)
{
    // First compute the 3 lines using Bresenham algorithm
    line_discrete line_table[3];
    bresenham(p0,p1,line_table[0]);
    bresenham(p1,p2,line_table[1]);
    bresenham(p2,p0,line_table[2]);

    // Then compute the 3 interpolation coefficient for each of the 3 lines
    line_interpolation_parameter coefficient_table[3];
    for(int k=0 ; k<3 ; ++k)
        coefficient_table[k] = line_interpolation_parameter(line_table[k]);

    // Perform the scanline algorithm on the 3 lines
    for(int k_line=0 ; k_line<3 ; ++k_line)
    {
        int const N = line_table[k_line].size();
        for(int k=0 ; k<N ; ++k)
        {

            ivec2 const& p    = line_table[k_line][k];             //current position on the line
            float const coefficient = coefficient_table[k_line][k];//current interpolation coefficient

            int const y = p.y();                                   //current y position

            //select the first and second vertex of the current line
            T const& first_param = select_first_parameter(param_0,param_1,param_2,k_line);
            T const& second_param = select_second_parameter(param_0,param_1,param_2,k_line);

            //compute the interpolated parameter at the current position
            T param;
            param = (1-coefficient)*first_param + coefficient*second_param;

            //prepare to store the position and the parameter as a position_info
            position_info<T> const info = {p,param};

            if( data.find(y) == data.end() ) // if an entry do not exists at the given y-value, setup left and right at the same value
                data[y] = info;
            else
                data[y].update(info);        // if an entry already exists, update the left or right value

        }
    }

}

template <typename T>
triangle_scanline_iterator<T> triangle_scanline<T>::begin() const { return data.cbegin(); }

template <typename T>
triangle_scanline_iterator<T> triangle_scanline<T>::end() const { return data.cend(); }

template <typename T>
triangle_scanline_iterator<T> triangle_scanline<T>::cbegin() const { return data.cbegin(); }

template <typename T>
triangle_scanline_iterator<T> triangle_scanline<T>::cend() const { return data.cend(); }

template <typename T>
std::ostream& operator<<(std::ostream& stream,triangle_scanline<T> const& scanline)
{
    for(auto const& value : scanline)
        stream<<"x="<<value.first<<" , "<<value.second<<std::endl;
    return stream;
}

template <typename T>
triangle_scanline<T> triangle_scanline_factory(ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,
                                               T const& param_0,T const& param_1,T const& param_2)
{
    return triangle_scanline<T>(p0,p1,p2,param_0,param_1,param_2);
}


}

#endif
