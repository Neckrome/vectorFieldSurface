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


#include "drawer.hpp"

#include "../discrete/ivec2.hpp"
#include "../discrete/line_discrete.hpp"
#include "../discrete/line_interpolation_parameter.hpp"
#include "../discrete/bresenham.hpp"
#include "../discrete/triangle_scanline.hpp"
#include "image.hpp"
#include "image_zbuffer.hpp"
#include "color.hpp"
#include "../lib/3d/vec2.hpp"
#include "../lib/Fragment.hpp"
#include "../lib/3d/vec3.hpp"
#include "texture.hpp"
#include <vector>

namespace cpe
{


void draw_line(image& im,ivec2 const& p0,ivec2 const& p1,color const& c)
{
    line_discrete line;
    bresenham(p0, p1, line);
    //std::cout << line << std::endl;
    for(const auto& point : line){
        im(point) = c;
    }
}

void draw_line(image& im,ivec2 const& p0,ivec2 const& p1,color const& c0,color const& c1)
{

    line_discrete line;
    bresenham(p0, p1, line);
    line_interpolation_parameter interpolation(line);

    for(int k = 0; k < line.size(); ++k){
        color c = (1 - interpolation[k]) * c0 + interpolation[k] * c1;
        im(line[k]) = c;
    }
}


void draw_line(image& im,ivec2 const& p0,ivec2 const& p1,color const& c0,color const& c1, float const& z0, float const& z1, vec2 const& uv0, vec2 const& uv1, texture const& tex, image_zbuffer& zbuffer)
{

    line_discrete line;
    bresenham(p0,p1,line);
    
    line_interpolation_parameter interpolation(line);
    for(int k=0 ; k < line.size() ; k++)
    {
        double alpha = interpolation[k];
        color c = (1 - alpha)*c0 + alpha * c1; 
        float z = (1 - alpha)*z0 + alpha * z1; 
        vec2 uv = (1 - alpha)*uv0 + alpha * uv1; 
        draw_point(im, zbuffer, line[k], z, c*tex(uv));
    }  
}



void draw_triangle(image& im,
                   ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,color const& c)
{
    auto scanline = triangle_scanline_factory(p0,p1,p2 ,c,c,c);

    for(auto const& value : scanline){

        auto const& left = value.second.left;
        auto const& right = value.second.right;
        ivec2 const& p_left = left.coordinate;
        ivec2 const& p_right = right.coordinate;

        draw_line(im, p_left, p_right, c);
    }
}

void draw_triangle_wireframe(image& im, ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,color const& c){
    draw_line(im, p0, p1, c);
    draw_line(im, p1, p2, c);
    draw_line(im, p2, p0, c);
}

void draw_triangle(image& im,
                   ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,
                   color const& c0,color const& c1,color const& c2)
{
    auto scanline = triangle_scanline_factory(p0,p1,p2,c0,c1,c2);

    for(auto const& value : scanline){

        auto const& left = value.second.left;
        auto const& right = value.second.right;
        ivec2 const& p_left = left.coordinate;
        ivec2 const& p_right = right.coordinate;
        color const& param_left = left.parameter;
        color const& param_right = right.parameter;

        draw_line(im, p_left, p_right, param_left, param_right);
    }

}

void draw_line(image& im,ivec2 const& p0,ivec2 const& p1,color const& c0,color const& c1, float z0, float z1, image_zbuffer& zbuffer){
    line_discrete line;
    bresenham(p0, p1, line);
    line_interpolation_parameter interpolation(line);

    for(int k = 0; k < line.size(); ++k){
        color c = (1 - interpolation[k]) * c0 + interpolation[k] * c1;
        float z = (1 - interpolation[k]) * z0 + interpolation[k] * z1;
        draw_point(im, zbuffer, line[k], z, c);
    }
}

void draw_point(image& im,image_zbuffer& zbuffer,ivec2 const& p,float const z,color const& c)
{
    if(p.x()<0 || p.x()>=im.Nx())
        return ;
    if(p.y()<0 || p.y()>=im.Ny())
        return ;


    if(z >= -1 && z <= 1 && z < zbuffer(p)){
        zbuffer(p) = z;
        im(p) = c;
    }
}



void draw_triangle(image& im,image_zbuffer& zbuffer,
                   ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,
                   color const& c0,color const& c1,color const& c2,
                   float z0,float z1,float z2)
{

    auto scanlineC = triangle_scanline_factory(p0,p1,p2 ,c0,c1,c2);
    auto scanlineZ = triangle_scanline_factory(p0,p1,p2 ,z0,z1,z2);

    auto itC = scanlineC.begin();
    auto itZ = scanlineZ.begin();
    auto it_end = scanlineC.end();
    for( ; itC!=it_end ; ++itC)
    {

        auto const& left = itC->second.left;
        auto const& right = itC->second.right;
        ivec2 const& p_left = left.coordinate;
        ivec2 const& p_right = right.coordinate;
        color const& param_left = left.parameter;
        color const& param_right = right.parameter;

        auto const& z_left = itZ->second.left;
        auto const& z_right = itZ->second.right;
        float const& z_param_left = z_left.parameter;
        float const& z_param_right = z_right.parameter;


        draw_line(im, p_left, p_right, param_left, param_right, z_param_left, z_param_right, zbuffer);
        ++itZ;
    }

    /*************************************
    // TO DO
    /*************************************
    // Construire scanline avec couleur
    // Construire scanline avec profondeur
    // Parcourir les deux scanline (utilisez les iterateurs)
    //   pa <- point gauche
    //   pb <- point droit
    //
    //   ca <- couleur a gauche
    //   cb <- couleur a droite
    //
    //   za <- profondeur a gauche
    //   zb <- profondeur a droite
    //
    //   affiche ligne entre pa et pb
    //     de couleur variant entre ca et cb
    //     de pronfondeur variant entre za et zb
    */

}



void draw_triangle(image& im,image_zbuffer& zbuffer,
                   ivec2 const& p0,ivec2 const& p1,ivec2 const& p2,
                   color const& c0,color const& c1,color const& c2,
                   vec2 const& uv0,vec2 const& uv1,vec2 const& uv2, texture const& tex,
                   float z0,float z1,float z2)
{

    auto scanline_color = triangle_scanline_factory(p0, p1, p2, c0, c1, c2);
    auto scanline_depth = triangle_scanline_factory(p0, p1, p2, z0, z1, z2);
    auto scanline_texture = triangle_scanline_factory(p0, p1, p2, uv0, uv1, uv2);

    auto it_color = scanline_color.begin();
    auto it_depth = scanline_depth.begin();
    auto it_texture = scanline_texture.begin();
    auto it_end_color = scanline_color.end();

    for( ; it_color!=it_end_color ; ++it_color)
    {
        auto const& left_color = it_color->second.left;
        auto const& left_depth = it_depth->second.left;
        auto const& left_texture = it_texture->second.left;

        auto const& right_color = it_color->second.right;
        auto const& right_depth = it_depth->second.right;
        auto const& right_texture = it_texture->second.right;

        auto const& p_left = left_color.coordinate;
        auto const& p_right = right_color.coordinate;

        auto const& param_left_color = left_color.parameter;
        auto const& param_right_color = right_color.parameter;

        auto const& param_left_depth = left_depth.parameter;
        auto const& param_right_depth = right_depth.parameter;

        auto const& param_left_texture = left_texture.parameter;
        auto const& param_right_texture = right_texture.parameter;
        
        draw_line(im, p_left, p_right, param_left_color, param_right_color, param_left_depth, param_right_depth, param_left_texture, param_right_texture, tex, zbuffer);
        ++it_depth;
        ++it_texture;
    }
}


void frag_line(std::vector<Fragment>& list_fragment, ivec2 const& p0,ivec2 const& p1,color const& c0,color const& c1)
{
    //Obtient les fragments le long d'une ligne
    line_discrete line;
    bresenham(p0, p1, line);
    line_interpolation_parameter interpolation(line);

    for(int k = 0; k < line.size(); ++k){
        double alpha = interpolation[k];
        color c = (1 - alpha) * c0 + alpha * c1;

        // Crée le fragment avec ses coordonnees ecran et barycentriques
        Fragment frag = Fragment(line[k].x(), line[k].y(), c.r(), c.g(), c.b());
        list_fragment.push_back(frag);
    }
}


void get_fragments(std::vector<Fragment>& list_fragment, ivec2 u0, ivec2 u1, ivec2 u2){
    // Permet d'optenir les paramètres d'interpolation
    auto scanline = triangle_scanline_factory(u0, u1, u2,
                                              color(1.0f, 0.0f, 0.0f), 
                                              color(0.0f, 1.0f, 0.0f), 
                                              color(0.0f, 0.0f, 1.0f));
    for(auto const& value : scanline){

        auto const& left = value.second.left;
        auto const& right = value.second.right;
        ivec2 const& p_left = left.coordinate;
        ivec2 const& p_right = right.coordinate;
        color const& param_left = left.parameter;
        color const& param_right = right.parameter;

        frag_line(list_fragment,p_left, p_right, param_left, param_right);
    }
}

}
