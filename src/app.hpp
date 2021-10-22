#ifndef DEF_APP
#define DEF_APP
 
#include <iostream>
#include <string>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

 
class App
{
    public:
 
    App();
    void run();
    static void callback();
 
    // Static variables used for the callback function
    static std::unique_ptr<VertexPositionGeometry> geometryFlat;
    static std::unique_ptr<VertexPositionGeometry> newGeometry;
    static std::unique_ptr<ManifoldSurfaceMesh> meshFlat;
    static std::unique_ptr<FaceData<Vector3>> normals;
    static std::unique_ptr<VertexData<Vector3>> bLoopData;
    static polyscope::SurfaceMesh* psReconsMesh;
    static VertexData<Vector3> debugGradient;
    static float lr;
    static float nw;
};
 
#endif