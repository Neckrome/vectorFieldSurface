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

App::App() {
 
}
 
void App::run() {
    std::cout << "Running..." << std::endl;
    
    // Creates a planar mesh with height values
    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = Utils::createMeshPlane(30, 30, 1, 1, [](float x, float y)->float{return x*x*y;});

    // Creates a planar mesh
    std::unique_ptr<ManifoldSurfaceMesh> meshFlat;
    std::unique_ptr<VertexPositionGeometry> geometryFlat;
    std::tie(meshFlat, geometryFlat) = Utils::createMeshPlane(30, 30, 1, 1, [](float x, float y)->float{return 0.0f;});

    // Gives projected normals
    std::unique_ptr<FaceData<Vector3>> projectedPoints;
    std::unique_ptr<FaceData<Vector3>> projectedNormals;
    std::tie(projectedPoints, projectedNormals) = Utils::getProjectedNormals(*mesh, *geometry);

    // Gives normals from projected normals
    std::unique_ptr<FaceData<Vector3>> normals = Utils::getNormals(*projectedNormals);

    // Deforms a planar mesh so that the geometry matches given normals
    std::unique_ptr<VertexPositionGeometry> newGeometry = DeformingMesh::solve(*meshFlat, *geometryFlat, *normals);

    // Visualization with polyscope
    polyscope::init();

    polyscope::SurfaceMesh* psMesh = polyscope::registerSurfaceMesh("Surface Mesh", geometry->vertexPositions, mesh->getFaceVertexList());
    polyscope::SurfaceMesh* psReconsMesh = polyscope::registerSurfaceMesh("Reconstructed Surface Mesh", newGeometry->vertexPositions, mesh->getFaceVertexList());

    polyscope::PointCloud* psPC = polyscope::registerPointCloud("Projected Points", *projectedPoints);
    psPC->addVectorQuantity("Projected Normals", *projectedNormals);
    psPC->addVectorQuantity("Normals", *normals);

    polyscope::show();

}