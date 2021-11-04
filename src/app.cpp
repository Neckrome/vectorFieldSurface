#include "app.hpp"
#include "geometry/utils.hpp"
#include "geometry/plane.hpp"
#include "test.hpp"
#include "optim/deformingMesh.hpp"
#include <functional>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"


#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"


App::App() {
 
}


void App::run() {
    std::cout << "Running..." << std::endl;
    
    Test test = Test();
    test.test1();

}