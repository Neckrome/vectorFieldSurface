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
 
    static std::unique_ptr<VertexPositionGeometry> geometryFlat;
    static std::unique_ptr<VertexPositionGeometry> newGeometry;
    static std::unique_ptr<ManifoldSurfaceMesh> meshFlat;
    static std::unique_ptr<FaceData<Vector3>> normals;
    static polyscope::SurfaceMesh* psReconsMesh;
};
 
#endif