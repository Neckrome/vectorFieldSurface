#ifndef DEF_UTILS
#define DEF_UTILS
 
#include <iostream>
#include <string>
#include <math.h>
#include <functional>
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "plane.hpp"

using namespace geometrycentral;
using namespace geometrycentral::surface;

 
class Utils
{
    public:
 
    Utils();
    static std::tuple<std::unique_ptr<ManifoldSurfaceMesh>, std::unique_ptr<VertexPositionGeometry>> createMeshPlane(int const Nx, int const Ny, float sizeX = 1.0f, float sizeY = 1.0f, std::function<float(float,float)> func = [](float x, float y)->float{return 1.0f;});
    static std::vector<Vector3> createPointsPlane(int const Nx, int const Ny, float sizeX = 1.0f, float sizeY = 1.0f, std::function<float(float,float)> func = [](float x, float y)->float{return 1.0f;});
    
    static std::tuple<std::unique_ptr<FaceData<Vector3>>, std::unique_ptr<FaceData<Vector3>>> getProjectedNormals(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry);
    static std::tuple<std::unique_ptr<FaceData<Vector3>>, std::unique_ptr<FaceData<Vector3>>> getProjectedNormals(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, Plane plane);
    static std::tuple<std::unique_ptr<VertexData<Vector3>>, std::unique_ptr<VertexData<Vector3>>> getProjectedVertexNormals(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry);

    static std::unique_ptr<FaceData<Vector3>> getNormals(FaceData<Vector3>& projectedNormals);
    static std::unique_ptr<FaceData<Vector3>> getNormals(FaceData<Vector3>& projectedNormals, Plane plane);

    static void centerPoints(VertexPositionGeometry& geometry);

    static std::unique_ptr<BoundaryLoop> getBoundaryLoop(ManifoldSurfaceMesh& mesh);
    static std::unique_ptr<VertexData<Vector3>> setBoundaryPositions(BoundaryLoop& bLoop, ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry);
    static std::unique_ptr<VertexData<Vector3>> setBoundaryPositions(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry);

    static std::unique_ptr<FaceData<Vector3>> getFaceNormalsFromVertexNormals(VertexData<Vector3>& vertexNormals, ManifoldSurfaceMesh& mesh);
    
    static std::tuple<std::unique_ptr<ManifoldSurfaceMesh>, std::unique_ptr<VertexPositionGeometry>> createIcoSphere(int level);
    static void twitchNormals(FaceData<Vector3>& normals);


};
 
#endif