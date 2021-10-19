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
    static std::unique_ptr<VertexPositionGeometry> solve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, FaceData<Vector3>& normals);
    
    private:

    
};
 
#endif