#ifndef DEF_DEFORMINGMESH
#define DEF_DEFORMINGMESH
 
#include <iostream>
#include <string>
#include <math.h>
#include <functional>
#include "geometrycentral/surface/vertex_position_geometry.h"


using namespace geometrycentral;
using namespace geometrycentral::surface;

 
class DeformingMesh
{
    public:
 
    DeformingMesh();
    static std::unique_ptr<VertexPositionGeometry> iterativeSolve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, VertexPositionGeometry& origGeom, FaceData<Vector3>& normals, VertexData<Vector3>& debugGradient, float lr, float nw);
    static std::unique_ptr<VertexPositionGeometry> iterativeSolve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, VertexPositionGeometry& origGeom, VertexData<Vector3>& bondaryFixedValues, FaceData<Vector3>& normals, VertexData<Vector3>& debugGradient, float lr, float nw);

    static std::unique_ptr<VertexPositionGeometry> analyticSolve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, FaceData<Vector3>& normals);
    
    private:

    
};
 
#endif