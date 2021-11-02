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

#include "line_interpolation_parameter.hpp"
#include "line_discrete.hpp"
#include "../lib/common/error_handling.hpp"
#include <cmath>


namespace cpe
{

line_interpolation_parameter::line_interpolation_parameter()
    :line_basic<float>()
{}

line_interpolation_parameter::line_interpolation_parameter(line_discrete const& line)
    :line_basic<float>()
{
    //special cases
    if(line.size()==0)
        return ;
    if(line.size()==1)
    {
        data.push_back(0.0f);
        return ;
    }

    // compute the full length
    ivec2 const& u0=line[0];             //first point
    ivec2 const& u1=line[line.size()-1]; //last point

    float const x0 = u0.x();
    float const y0 = u0.y();

    float const x1 = u1.x();
    float const y1 = u1.y();

    float dL0=std::sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
    ASSERT_CPE(dL0>1e-5,"too small difference");

    //for each values of the line, compute the relative length
    int const N = line.size();
    for(int k=0 ; k<N ; ++k)
    {
        ivec2 const& u = line[k];

        float const x = u.x();
        float const y = u.y();

        float const dx = x-x0;
        float const dy = y-y0;

        float const dL = std::sqrt(dx*dx+dy*dy);
        float const alpha = dL/dL0;

        data.push_back(alpha);
    }

    //sanity check
    ASSERT_CPE(static_cast<int>(data.size())==static_cast<int>(line.size()),"Problem of size integrity");

}






}
