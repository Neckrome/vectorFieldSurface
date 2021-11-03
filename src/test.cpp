#include "test.hpp"
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

#include <Eigen/Sparse>


// Static variables declaration
std::unique_ptr<VertexPositionGeometry> Test::geometryFlat;
std::unique_ptr<VertexPositionGeometry> Test::newGeometry;
std::unique_ptr<ManifoldSurfaceMesh> Test::meshFlat;
std::unique_ptr<FaceData<Vector3>> Test::normals;
FaceData<Vector3> Test::refNormals;
std::unique_ptr<VertexData<Vector3>> Test::bLoopData;
polyscope::SurfaceMesh* Test::psReconsMesh;
VertexData<Vector3> Test::debugGradient;
float Test::lr = 0.04f;
float Test::nw = 1.0f;
int i = 0;
int cpt = 0;

Test::Test() {
 
}

bool adjacent(int i, int j, int W, int H){
    return (i-j == -1 || i-j == 1 || i-j == W || i-j == -W);
}

void Test::callback1(){
    // ImGui
    ImGui::PushItemWidth(100);

    ImGui::InputInt("Iterations : ", &i);            
    ImGui::SliderFloat("LR : ", &lr, 0.001f, 0.05f);
    ImGui::SliderFloat("Normal weight : ", &nw, 0.1f, 100.0f);

    ImGui::PopItemWidth();

    i++;
    newGeometry = DeformingMesh::iterativeSolve(*meshFlat, *newGeometry, *geometryFlat, *bLoopData, *normals, debugGradient, lr, nw);
    Utils::centerPoints(*newGeometry);
    psReconsMesh->updateVertexPositions(newGeometry->vertexPositions);
    psReconsMesh->addVertexVectorQuantity("Debug Gradient", debugGradient);

}

void Test::callback2(){
    // ImGui
    ImGui::PushItemWidth(100);

    ImGui::InputInt("Iterations : ", &i);            
    ImGui::SliderFloat("LR : ", &lr, 0.001f, 0.05f);
    ImGui::SliderFloat("Normal weight : ", &nw, 0.1f, 100.0f);

    ImGui::PopItemWidth();

    i++;
    newGeometry = DeformingMesh::iterativeSolve(*meshFlat, *newGeometry, *geometryFlat, refNormals, debugGradient, lr, nw);
    //Utils::centerPoints(*newGeometry);
    psReconsMesh->updateVertexPositions(newGeometry->vertexPositions);
    psReconsMesh->addVertexVectorQuantity("Debug Gradient", debugGradient);
    newGeometry->requireFaceNormals();
    psReconsMesh->addFaceVectorQuantity("Real Normals", newGeometry->faceNormals);
}

void Test::callback3(){
    // ImGui
    ImGui::PushItemWidth(100);

    ImGui::InputInt("Iterations : ", &i);            
    ImGui::SliderFloat("LR : ", &lr, 0.001f, 0.05f);
    ImGui::SliderFloat("Normal weight : ", &nw, 0.1f, 100.0f);

    ImGui::PopItemWidth();

    /*
    int W = 10, H = 10;
    // Laplacian Solve
    double t = 1000.0;
    Eigen::SparseMatrix<double> lap(W*H, W*H);
    Eigen::SparseMatrix<double> Id(W*H, W*H);
    Id.setIdentity();

    Eigen::VectorXd u_0 = Eigen::VectorXd::Zero(W*H);
    Eigen::VectorXd phi_0 = Eigen::VectorXd::Zero(W*H);
    Eigen::VectorXd v_x0 = Eigen::VectorXd::Zero(W*H);
    Eigen::VectorXd v_y0 = Eigen::VectorXd::Zero(W*H);

    v_x0[27] = 1.0;
    v_y0[27] = 1.0;
    u_0[27] = std::sqrt(2.0);
    phi_0[27] = 1.0;

    v_x0[72] = -1.0;
    v_y0[72] = -1.0;
    u_0[72] = std::sqrt(3.0);
    phi_0[72] = 1.0;

    double theta = 2*3.141592*double(cpt++)/20;
    double x = cos(theta);
    double y = sin(theta);
    v_x0[11] = x;
    v_y0[11] = y;
    u_0[11] = std::sqrt(1.0);
    phi_0[11] = 1.0;

    for(auto i=0; i < W*H; ++i){
        for(auto j=0; j < W*H; ++j){
            if(i == j)
                lap.coeffRef(i, j) = -4.0;
            if(adjacent(i,j,W,H))
                lap.coeffRef(i,j) = 1.0;
        }
    }
    
    Eigen::SparseMatrix<double> A = Id - t*lap;
    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> >   solver;
    solver.analyzePattern(A); 
    solver.factorize(A); 

    Eigen::VectorXd u_t = solver.solve(u_0);
    Eigen::VectorXd phi_t = solver.solve(phi_0);
    Eigen::VectorXd v_xt = solver.solve(v_x0);
    Eigen::VectorXd v_yt = solver.solve(v_y0);

    Eigen::VectorXd final_x(W*H);
    Eigen::VectorXd final_y(W*H);


    for(auto i=0; i < W*H; ++i){
        final_x[i] = v_xt[i] / std::sqrt((v_xt[i]*v_xt[i] + v_yt[i]*v_yt[i])) * u_t[i] / phi_t[i];
        final_y[i] = v_yt[i] / std::sqrt((v_xt[i]*v_xt[i] + v_yt[i]*v_yt[i])) * u_t[i] / phi_t[i];
    }

    // Fill
    std::vector<Vector3> final(W*H);
    int i = 0;
    for(Vector3& val : final){
        val = Vector3{final_x[i], 0.0f, final_y[i]};
        
        i++;
    }

    polyscope::getPointCloud("VF")->addVectorQuantity("Value", final);*/
    
}

void Test::test1() {
    
    // Creates a planar mesh with height values
    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = Utils::createMeshPlane(30, 30, 5, 5, [](float x, float y)->float{return x*x-y*y;});
    Utils::centerPoints(*geometry);
    geometry->requireVertexNormals();

    // Creates a planar mesh
    //std::unique_ptr<ManifoldSurfaceMesh> meshFlat;
    std::tie(meshFlat, geometryFlat) = Utils::createMeshPlane(30, 30, 5, 5, [](float x, float y)->float{return 0.0f;});

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

    polyscope::state::userCallback = Test::callback1;

    polyscope::show();

}

void Test::test2() {
    
    // Creates an icosphere and twitched its normals
    std::unique_ptr<ManifoldSurfaceMesh> meshIco;
    std::unique_ptr<VertexPositionGeometry> geometryIco;
    std::tie(meshIco, geometryIco) = Utils::createIcoSphere(2);
    geometryIco->requireFaceNormals();
    refNormals = geometryIco->faceNormals;
    Utils::twitchNormals(refNormals);

    std::tie(meshFlat, geometryFlat) = Utils::createIcoSphere(2);


    // Deforms an icosphere so that the geometry matches given normals
    debugGradient = VertexData<Vector3>(*meshIco);
    newGeometry = DeformingMesh::iterativeSolve(*meshFlat, *geometryFlat, *geometryFlat, refNormals, debugGradient, lr, nw);


    // Visualization with polyscope
    polyscope::init();

    psReconsMesh = polyscope::registerSurfaceMesh("Icosphere", newGeometry->vertexPositions, meshIco->getFaceVertexList());
    psReconsMesh->addFaceVectorQuantity("Real Normals", geometryIco->faceNormals);
    psReconsMesh->addFaceVectorQuantity("Twitched Normals", refNormals);
    std::cout << refNormals.size() << std::endl;

    polyscope::state::userCallback = Test::callback2;

    polyscope::show();

}



void Test::test3() {
    
    // Creates a 2D vector Field
    int W = 40, H = 40;
    
    std::vector<Vector3> pts = Utils::createPointsPlane(W, H);
    std::vector<Vector3> values(W*H);
    for(Vector3& val : values){
        float theta = 2*3.141592*polyscope::randomUnit();
        float x = cos(theta);
        float y = sin(theta);
        val = Vector3{x, 0.0f, y};
    }
    
    // Creates a planar mesh with height values
    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = Utils::createMeshPlane(W, H, 5, 5, [](float x, float y)->float{return sin(x)*sin(y);});
    Utils::centerPoints(*geometry);
    geometry->requireVertexNormals();
    geometry->requireFaceNormals();

    // Creates a planar mesh
    //std::unique_ptr<ManifoldSurfaceMesh> meshFlat;
    std::tie(meshFlat, geometryFlat) = Utils::createMeshPlane(W, H, 5, 5, [](float x, float y)->float{return 0.0f;});

    // Gives projected normals
    std::unique_ptr<VertexData<Vector3>> projectedPoints;
    std::unique_ptr<VertexData<Vector3>> projectedNormals;
    std::tie(projectedPoints, projectedNormals) = Utils::getProjectedVertexNormals(*mesh, *geometry);

    // Laplacian Solve
    double t = 0.01;
    Eigen::SparseMatrix<double> lap(W*H, W*H);
    Eigen::SparseMatrix<double> Id(W*H, W*H);
    Id.setIdentity();

    Eigen::VectorXd u_0 = Eigen::VectorXd::Zero(W*H);
    Eigen::VectorXd phi_0 = Eigen::VectorXd::Zero(W*H);
    Eigen::VectorXd v_x0 = Eigen::VectorXd::Zero(W*H);
    Eigen::VectorXd v_y0 = Eigen::VectorXd::Zero(W*H);

    std::vector<int> preservedNormalsId = {0,1,2,3,4,5,6,7,8,9,90,91,92,93,94,95,96,97,98,99,10,20,30,40,50,60,70,80,19,29,39,49,59,69,79,89};

    //for(int id : preservedNormalsId){
    for(int id = 0; id < 1600; ++id){
        v_x0[id] = (*projectedNormals)[id][0];
        v_y0[id] = (*projectedNormals)[id][2];
        u_0[id] = norm((*projectedNormals)[id]);
        phi_0[id] = 1.0;
    }
    /*
    v_x0[27] = 1.0;
    v_y0[27] = 1.0;
    u_0[27] = std::sqrt(2.0);
    phi_0[27] = 1.0;

    v_x0[72] = -1.0;
    v_y0[72] = -1.0;
    u_0[72] = std::sqrt(6.0);
    phi_0[72] = 1.0;

    double theta = 2*3.141592*double(cpt++)/360;
    double x = cos(theta);
    double y = sin(theta);
    v_x0[11] = x;
    v_y0[11] = y;
    u_0[11] = std::sqrt(1.0);
    phi_0[11] = 1.0; */

    for(auto i=0; i < W*H; ++i){
        for(auto j=0; j < W*H; ++j){
            if(i == j){
                lap.coeffRef(i, j) = -4.0;
                if(i == 0 || i == W*H-1 || j == 0 || j == W*H-1)
                    lap.coeffRef(i, j) = -3.0;
            }
                
            if(adjacent(i,j,W,H))
                lap.coeffRef(i,j) = 1.0;
        }
    }
    
    Eigen::SparseMatrix<double> A = Id - t*lap;
    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> >   solver;
    solver.analyzePattern(A); 
    solver.factorize(A); 

    Eigen::VectorXd u_t = solver.solve(u_0);
    Eigen::VectorXd phi_t = solver.solve(phi_0);
    Eigen::VectorXd v_xt = solver.solve(v_x0);
    Eigen::VectorXd v_yt = solver.solve(v_y0);

    Eigen::VectorXd final_x(W*H);
    Eigen::VectorXd final_y(W*H);


    for(auto i=0; i < W*H; ++i){
        final_x[i] = v_xt[i] / std::sqrt((v_xt[i]*v_xt[i] + v_yt[i]*v_yt[i])) * u_t[i] / phi_t[i];
        final_y[i] = v_yt[i] / std::sqrt((v_xt[i]*v_xt[i] + v_yt[i]*v_yt[i])) * u_t[i] / phi_t[i];
    }

    // Fill
    std::vector<Vector3> final(W*H);
    VertexData<Vector3> estimatedProjectedVertexNormals(*meshFlat);
    int i = 0;
    for(Vector3& val : final){
        val = Vector3{final_x[i], 0.0f, final_y[i]};
        estimatedProjectedVertexNormals[i] = Vector3{final_x[i], 0.0f, final_y[i]};
        i++;
    }

    std::unique_ptr<FaceData<Vector3>> estimatedProjectedFaceNormals = Utils::getFaceNormalsFromVertexNormals(estimatedProjectedVertexNormals, *meshFlat);

    // Gives normals from projected normals
    normals = Utils::getNormals(*estimatedProjectedFaceNormals);

    // Find boundary loop
    bLoopData = Utils::setBoundaryPositions(*mesh, *geometry);
    
    // Deforms a planar mesh so that the geometry matches given normals
    debugGradient = VertexData<Vector3>(*mesh);
    newGeometry = DeformingMesh::iterativeSolve(*meshFlat, *geometryFlat, *geometryFlat, *bLoopData, *normals, debugGradient, lr, nw);
    Utils::centerPoints(*newGeometry);
    newGeometry->requireFaceNormals();


    // Visualization with polyscope
    polyscope::init();

    polyscope::PointCloud* pointCloudVF = polyscope::registerPointCloud("VF", *projectedPoints);
    pointCloudVF->addVectorQuantity("Estimated Normals", final);
    pointCloudVF->addVectorQuantity("Real Normals", *projectedNormals);

    psReconsMesh = polyscope::registerSurfaceMesh("Reconstructed Mesh", newGeometry->vertexPositions, mesh->getFaceVertexList());
    psReconsMesh->addFaceVectorQuantity("Estimated normals", *normals);
    psReconsMesh->addFaceVectorQuantity("Groundtruth", geometry->faceNormals);
    psReconsMesh->addFaceVectorQuantity("Real normals", newGeometry->faceNormals);

    polyscope::SurfaceMesh* psMesh = polyscope::registerSurfaceMesh("Surface Mesh", geometry->vertexPositions, mesh->getFaceVertexList());


    //polyscope::state::userCallback = Test::callback3;

    polyscope::show();

}