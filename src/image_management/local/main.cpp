
/** TP 5ETI IMI- CPE Lyon - 2015-2016 */


/**
#include "lib/common/error_handling.hpp"

#include "discrete/bresenham.hpp"
#include "discrete/bresenham_octant.hpp"
#include "discrete/line_discrete.hpp"
#include "discrete/line_interpolation_parameter.hpp"
#include "image/image.hpp"
#include "image/io/image_ppm.hpp"
#include "image/drawer.hpp"
#include "discrete/triangle_scanline.hpp"
#include "image/image_zbuffer.hpp"
#include "lib/mesh/mesh.hpp"
#include "lib/mesh/mesh_io.hpp"
#include "render_engine/render_engine.hpp"
#include "../lib/3d/mat4.hpp"
#include "../lib/3d/mat3.hpp"
#include "image/texture.hpp"
#include <chrono> 

#include <iostream>
#include <fstream>
#include <cmath>

using namespace cpe;

int main(int argc,char *argv[]) 
{

    std::cout<<"***********************"<<std::endl;
    std::cout<<"run "<<argv[0]<<" with "<<argc-1<<" parameters ... \n"<<std::endl;


    try
    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        int const Nx = 1920;
        int const Ny = 1920;

        image im(Nx,Ny);
        //image_zbuffer zbuffer(Nx, Ny);
        im.fill({1.0f,1.0f,1.0f});

        //chargement du fichier
        mesh m = load_mesh_file("../projet/data/dino.obj");
        texture tex; // texture
        tex.load("../projet/data/dino.ppm");
        texture nm; //normal map
        nm.load("../projet/data/dino_nm.ppm");

        //applique potentiellement une rotation, translation
        // (scaling possible egalement)
        m.transform_apply_rotation({0,1,0},-M_PI/4.0f);
        m.fill_normal(); // recalcule les normales
        m.transform_apply_translation({0,0,-1.3f});


        image_zbuffer zbuffer(Nx, Ny);
        mat4 model; //identitee
        mat4 view; //identitee
        mat4 projection;
        //matrice de projection angle de vue de 60 degres,
        // image de taille carree,
        // sommets visibles entre z=0.1 et z=20.
        projection.set_projection_perspective(60*M_PI/180.0f,float(Nx)/Ny,0.1f,20.0f);

        start = std::chrono::system_clock::now(); //Pour mesurer le temps d'execution
        render(im, zbuffer, m, tex, nm, model, view, projection);
        end = std::chrono::system_clock::now();
        

        std::chrono::duration<double> elapsed_seconds = end - start; 
        std::cout << "Rendering time: " << elapsed_seconds.count() << std::endl;
        
        im.save("mon_image.ppm");
        
    }
    catch(cpe::exception_cpe const& e)
    {


        std::cout<<"**********************************"<<std::endl;
        std::cout<<"**********************************"<<std::endl;
        std::cout<<"Exception found"<<std::endl;
        std::cout<<e.info()<<std::endl;
        std::cout<<"\nSaving backtrace in file backtrace_log.txt"<<std::endl;
        std::cout<<"**********************************"<<std::endl;
        std::cout<<"**********************************"<<std::endl;


        std::ofstream back_file("backtrace_log.txt");
        back_file<<e.info()<<std::endl;
        back_file<<e.info_backtrace();
        back_file.close();

        exit(1);
    }


    std::cout<<"Exit Main"<<std::endl;

    return 0;
}
*/