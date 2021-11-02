#ifndef DEF_TEST
#define DEF_TEST

#include <iostream>
#include <string>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

 
class Test
{
    public:
 
    Test();
    void test1();
    void test2();
    void test3();
    static void callback1();
    static void callback2();
    static void callback3();
 
    // Static variables used for the callback function
    static std::unique_ptr<VertexPositionGeometry> geometryFlat;
    static std::unique_ptr<VertexPositionGeometry> newGeometry;
    static std::unique_ptr<ManifoldSurfaceMesh> meshFlat;
    static std::unique_ptr<FaceData<Vector3>> normals;
    static FaceData<Vector3> refNormals;
    static std::unique_ptr<VertexData<Vector3>> bLoopData;
    static polyscope::SurfaceMesh* psReconsMesh;
    static VertexData<Vector3> debugGradient;
    static float lr;
    static float nw;

    //test3
    
};
 
#endif