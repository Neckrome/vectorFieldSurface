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
std::unique_ptr<VertexData<Vector3>> App::bLoopData;
polyscope::SurfaceMesh* App::psReconsMesh;
VertexData<Vector3> App::debugGradient;
float App::lr = 0.004f;
float App::nw = 1.0f;
int i = 0;

App::App() {
 
}

void App::callback(){
    // ImGui
    ImGui::PushItemWidth(100); // Make ui elements 100 pixels wide,
                             // instead of full width. Must have 
                             // matching PopItemWidth() below.

    ImGui::InputInt("Iterations : ", &i);             // set a int variable
    ImGui::SliderFloat("LR : ", &lr, 0.001f, 10.0f);
    ImGui::SliderFloat("Normal weight : ", &nw, 0.05f, 10.0f);

    ImGui::PopItemWidth();

    i++;
    newGeometry = DeformingMesh::iterativeSolve(*meshFlat, *newGeometry, *geometryFlat, *bLoopData, *normals, debugGradient, lr, nw);
    //Utils::centerPoints(*newGeometry);
    psReconsMesh->updateVertexPositions(newGeometry->vertexPositions);
    psReconsMesh->addVertexVectorQuantity("Debug Gradient", debugGradient);
}
 
void App::run() {
    std::cout << "Running..." << std::endl;
    
    // Creates a planar mesh with height values
    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = Utils::createMeshPlane(30, 30, 10, 10, [](float x, float y)->float{return sin(x)*sin(y);});
    Utils::centerPoints(*geometry);
    geometry->requireVertexNormals();

    // Creates a planar mesh
    //std::unique_ptr<ManifoldSurfaceMesh> meshFlat;
    std::tie(meshFlat, geometryFlat) = Utils::createMeshPlane(30, 30, 10, 10, [](float x, float y)->float{return 0.0f;});

    // Gives projected normals
    std::unique_ptr<FaceData<Vector3>> projectedPoints;
    std::unique_ptr<FaceData<Vector3>> projectedNormals;
    std::tie(projectedPoints, projectedNormals) = Utils::getProjectedNormals(*mesh, *geometry);

    // Gives normals from projected normals
    normals = Utils::getNormals(*projectedNormals);

    // Find boundary loop
    bLoopData = Utils::setBoundaryPositions(*mesh, *geometry);
    
    // Deforms a planar mesh so that the geometry matches given normals
    debugGradient = VertexData<Vector3>(*mesh);
    newGeometry = DeformingMesh::iterativeSolve(*meshFlat, *geometryFlat, *geometryFlat, *bLoopData, *normals, debugGradient, lr, nw);


    // Visualization with polyscope
    polyscope::init();

    polyscope::SurfaceMesh* psMesh = polyscope::registerSurfaceMesh("Surface Mesh", geometry->vertexPositions, mesh->getFaceVertexList());
    psMesh->addVertexVectorQuantity("Vertex Normals", geometry->vertexNormals);

    psReconsMesh = polyscope::registerSurfaceMesh("Reconstructed Surface Mesh", newGeometry->vertexPositions, mesh->getFaceVertexList());
    psReconsMesh->addFaceVectorQuantity("Normals", *normals);
    psReconsMesh->addVertexVectorQuantity("Debug Gradient", debugGradient);

    polyscope::PointCloud* psPC = polyscope::registerPointCloud("Projected Points", *projectedPoints);
    psPC->addVectorQuantity("Projected Normals", *projectedNormals);
    psPC->addVectorQuantity("Normals", *normals);

    polyscope::state::userCallback = App::callback;

    polyscope::show();

}