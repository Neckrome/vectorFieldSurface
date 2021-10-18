#include "app.hpp"
#include "geometry/utils.hpp"
#include "geometry/plane.hpp"
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
    
    // Create a planar mesh with height values
    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = Utils::createMeshPlane(3, 3, 10, 10, [](float x, float y)->float{return 0.1f*x*x * sin(y);});

    // Give projected normals
    std::unique_ptr<std::vector<Vector3>> projectedPoints;
    std::unique_ptr<std::vector<Vector3>> projectedNormals;
    std::tie(projectedPoints, projectedNormals) = Utils::getProjectedNormals(*mesh, *geometry);

    // Give normals from projected normals
    std::unique_ptr<std::vector<Vector3>> normals = Utils::getNormals(*projectedNormals);

    polyscope::init();

    polyscope::SurfaceMesh* psMesh = polyscope::registerSurfaceMesh("Surface Mesh", geometry->vertexPositions, mesh->getFaceVertexList());

    polyscope::PointCloud* psPC = polyscope::registerPointCloud("Projected Points", *projectedPoints);
    psPC->addVectorQuantity("Projected Normals", *projectedNormals);
    psPC->addVectorQuantity("Normals", *normals);

    polyscope::show();

}