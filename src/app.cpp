#include "app.hpp"
#include "geometry/utils.hpp"
#include "geometry/plane.hpp"
#include "optim/deformingMesh.hpp"
#include <functional>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"


#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

// Static variables declaration
std::unique_ptr<VertexPositionGeometry> App::geometryFlat;
std::unique_ptr<VertexPositionGeometry> App::newGeometry;
std::unique_ptr<ManifoldSurfaceMesh> App::meshFlat;
std::unique_ptr<FaceData<Vector3>> App::normals;
polyscope::SurfaceMesh* App::psReconsMesh;
int i = 0;

App::App() {
 
}

void App::callback(){
    std::cout << i++ << std::endl;
    newGeometry = DeformingMesh::iterativeSolve(*meshFlat, *newGeometry, *geometryFlat, *normals);
    Utils::centerPoints(*newGeometry);
    psReconsMesh->updateVertexPositions(newGeometry->vertexPositions);
}
 
void App::run() {
    std::cout << "Running..." << std::endl;
    
    // Creates a planar mesh with height values
    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = Utils::createMeshPlane(20, 20, 1, 1, [](float x, float y)->float{return x*x-y*y;});
    Utils::centerPoints(*geometry);

    // Creates a planar mesh
    //std::unique_ptr<ManifoldSurfaceMesh> meshFlat;
    std::tie(meshFlat, geometryFlat) = Utils::createMeshPlane(20, 20, 1, 1, [](float x, float y)->float{return 0.0f;});

    // Gives projected normals
    std::unique_ptr<FaceData<Vector3>> projectedPoints;
    std::unique_ptr<FaceData<Vector3>> projectedNormals;
    std::tie(projectedPoints, projectedNormals) = Utils::getProjectedNormals(*mesh, *geometry);

    // Gives normals from projected normals
    normals = Utils::getNormals(*projectedNormals);

    // Find boundary loop
    std::unique_ptr<BoundaryLoop> bLoop = Utils::getBoundaryLoop(*mesh);
    std::unique_ptr<BoundaryLoopData<Vector3>> bLoopData= Utils::setBoundaryPositions();
    
    // Deforms a planar mesh so that the geometry matches given normals
    newGeometry = DeformingMesh::iterativeSolve(*meshFlat, *geometryFlat, *geometryFlat, *normals);

    // Visualization with polyscope
    polyscope::init();

    polyscope::SurfaceMesh* psMesh = polyscope::registerSurfaceMesh("Surface Mesh", geometry->vertexPositions, mesh->getFaceVertexList());
    psReconsMesh = polyscope::registerSurfaceMesh("Reconstructed Surface Mesh", newGeometry->vertexPositions, mesh->getFaceVertexList());

    polyscope::PointCloud* psPC = polyscope::registerPointCloud("Projected Points", *projectedPoints);
    psPC->addVectorQuantity("Projected Normals", *projectedNormals);
    psPC->addVectorQuantity("Normals", *normals);

    polyscope::state::userCallback = App::callback;

    polyscope::show();

}