#include "utils.hpp"
#include "plane.hpp"
#include <vector>
#include <array>
#include <tuple>
#include <functional>
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_mesh_factories.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

Utils::Utils() {
 
}
 
std::tuple<std::unique_ptr<ManifoldSurfaceMesh>, std::unique_ptr<VertexPositionGeometry>> Utils::createMeshPlane(int const Nx, int const Ny, float sizeX, float sizeY, std::function<float(float,float)> func) {
    /**
    Creates a planar mesh grid with an elevation following a function
    @param Nx, Ny : number of points on a x axis (resp. z axis)
    @param sizeX, sizeY : size of the mesh on x axis (resp. z axis)
    @param func : elevation function followed by the grid
    @return the mesh and its geometry
    */
    std::vector<Vector3> pts(Nx*Ny);
    
    int i = 0;
    for(float x = -0.5*sizeX; x < 0.5*sizeX + sizeX/((Nx-1)*2); x += sizeX/(Nx-1)){
        for(float y = -0.5*sizeY; y < 0.5*sizeY + sizeY/((Ny-1)*2); y += sizeY/(Ny-1)){
            pts[i++] = {x, func(x,y), y};
        }
    }

    
    std::vector<std::vector<size_t>> faces(2*(Nx-1)*(Ny-1), std::vector<size_t>(3, 0));

    for(int x = 0; x < Nx-1; ++x){
        for(int y = 0; y < Ny-1; ++y){
            faces[2*(x*(Ny-1)+y)] = {size_t(x*Ny+y), size_t(x*Ny+y+1), size_t((x+1)*Ny+y)};
            faces[2*(x*(Ny-1)+y)+1] = {size_t(x*Ny+y+1), size_t((x+1)*Ny+y+1), size_t((x+1)*Ny+y)};
        }
    }

    return makeManifoldSurfaceMeshAndGeometry(faces, pts);
}

 std::tuple<std::unique_ptr<ManifoldSurfaceMesh>, std::unique_ptr<VertexPositionGeometry>> Utils::createPointsPlane(int const Nx, int const Ny, float sizeX, float sizeY, std::function<float(float,float)> func) {
    /**
    TODO
    */
    std::vector<Vector3> pts(Nx*Ny);
    
    int i = 0;
    for(float x = -0.5*sizeX; x < 0.5*sizeX + sizeX/((Nx-1)*2); x += sizeX/(Nx-1)){
        for(float y = -0.5*sizeY; y < 0.5*sizeY + sizeY/((Ny-1)*2); y += sizeY/(Ny-1)){
            pts[i++] = {x, func(x,y), y};
        }
    }

    
    std::vector<std::vector<size_t>> faces(2*(Nx-1)*(Ny-1), std::vector<size_t>(3, 0));

    for(int x = 0; x < Nx-1; ++x){
        for(int y = 0; y < Ny-1; ++y){
            faces[2*(x*(Ny-1)+y)] = {size_t(x*Ny+y), size_t(x*Ny+y+1), size_t((x+1)*Ny+y)};
            faces[2*(x*(Ny-1)+y)+1] = {size_t(x*Ny+y+1), size_t((x+1)*Ny+y+1), size_t((x+1)*Ny+y)};
        }
    }

    return makeManifoldSurfaceMeshAndGeometry(faces, pts);
}

std::tuple<std::unique_ptr<FaceData<Vector3>>, std::unique_ptr<FaceData<Vector3>>> Utils::getProjectedNormals(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry){
    return Utils::getProjectedNormals(mesh, geometry, Plane(Vector3{0.0f, 1.0f, 0.0f}));
}

std::tuple<std::unique_ptr<FaceData<Vector3>>, std::unique_ptr<FaceData<Vector3>>> Utils::getProjectedNormals(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, Plane plane){
    /**
    Projects normals of a mesh on a plane
    @param mesh : connectivity of the mesh
    @param geometry : positions of the vertices of the mesh in space
    @param plane : the plane on which normals are projected
    @return  positions and values the projected normals
    */
    std::unique_ptr<FaceData<Vector3>> pts = std::make_unique<FaceData<Vector3>>(); (*pts) = FaceData<Vector3>(mesh);
    std::unique_ptr<FaceData<Vector3>> normals = std::make_unique<FaceData<Vector3>>(); (*normals) = FaceData<Vector3>(mesh);
    
    for(Face f : mesh.faces()){
        Vector3 p0 = geometry.inputVertexPositions[f.halfedge().vertex()];
        Vector3 p1 = geometry.inputVertexPositions[f.halfedge().next().vertex()];
        Vector3 p2 = geometry.inputVertexPositions[f.halfedge().next().next().vertex()];
        Vector3 normal = cross(p1-p0, p2-p1); normal /= norm(normal);
        
        (*normals)[f] = plane.projection(normal);
        (*pts)[f] = plane.projection((p0 + p1 + p2)/3);
    }

    return std::tuple<std::unique_ptr<FaceData<Vector3>>, std::unique_ptr<FaceData<Vector3>>>(std::move(pts), std::move(normals));
}

std::unique_ptr<FaceData<Vector3>> Utils::getNormals(FaceData<Vector3>& projectedNormals){
    return Utils::getNormals(projectedNormals, Plane(Vector3{0.0f, 1.0f, 0.0f}));
}

std::unique_ptr<FaceData<Vector3>> Utils::getNormals(FaceData<Vector3>& projectedNormals, Plane plane){
    /**
    Retrieve normals from projected normals
    @param projectedNormals : projected normals on a plane
    @param plane : plane on which normals were projected
    @return normals
    */
    std::unique_ptr<FaceData<Vector3>> normals = std::make_unique<FaceData<Vector3>>();
    (*normals) = projectedNormals;

    for(size_t i = 0; i < projectedNormals.size(); ++i){
        Vector3 p_n = projectedNormals[i];
        (*normals)[i] = p_n + sqrt(1 - norm(p_n)*norm(p_n)) * plane.n;
    }

    return normals;
}

void Utils::centerPoints(VertexPositionGeometry& geometry){
    /**
    Center a mesh at the origin
    @param geometry : vertices to be centered
    */
    Vector3 mean;
    for(size_t i = 0; i < geometry.vertexPositions.size(); ++i){
        mean += geometry.vertexPositions[i];
    }
    mean /= geometry.vertexPositions.size();

    for(size_t i = 0; i < geometry.vertexPositions.size(); ++i){
        geometry.vertexPositions[i] -= mean;
    }
}

std::unique_ptr<BoundaryLoop> Utils::getBoundaryLoop(ManifoldSurfaceMesh& mesh){
    /**
    Get the first boundary loop found on a mesh
    @param mesh : the mesh were boundary loops are looked at
    @return the boundary loop
    */
    std::unique_ptr<BoundaryLoop> BL = std::make_unique<BoundaryLoop>();

    for(Halfedge he : mesh.halfedges()){
        if(he.face().isBoundaryLoop() == true){
            *BL = he.face().asBoundaryLoop();
            break;
        }
    }
    return BL;
}

std::unique_ptr<VertexData<Vector3>> Utils::setBoundaryPositions(BoundaryLoop& bLoop, ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry){
    /**
    Set vertex positions on vertices of the boundary loop
    @param bLoop : boundary loop
    @param mesh : the mesh to construct the VertexData container
    @param geometry : contains vertices positions
    @return vertex data containing vertex positions on the boundary loop
    */
    std::unique_ptr<VertexData<Vector3>> bLoopVertices = std::make_unique<VertexData<Vector3>>();
    *bLoopVertices = VertexData<Vector3>(mesh); bLoopVertices->fill(Vector3{0.0f, 0.0f, 0.0f});

    for(Vertex v : bLoop.adjacentVertices()){
        (*bLoopVertices)[v] = geometry.inputVertexPositions[v];
    }

    return bLoopVertices;
}

std::unique_ptr<VertexData<Vector3>> Utils::setBoundaryPositions(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry){
    /**
    Set vertex positions on vertices of the boundary loop
    @param mesh : the mesh to construct the VertexData container
    @param geometry : contains vertices positions
    @return vertex data containing vertex positions on the boundary loop
    */
    std::unique_ptr<BoundaryLoop> bLoop = Utils::getBoundaryLoop(mesh);
    return Utils::setBoundaryPositions(*bLoop, mesh, geometry);
}

std::unique_ptr<FaceData<Vector3>> Utils::getFaceNormalsFromVertexNormals(VertexData<Vector3> vertexNormals, ManifoldSurfaceMesh& mesh){
    /**
    Interpolate face normals knowing vertex normals. Defined as the corner-angle weighted average of incident vertices normals
    @param vertexNormals : normals defined on each vertex
    @param mesh : describes the angles
    @return face normals
    */
    std::unique_ptr<FaceData<Vector3>> normals = std::make_unique<FaceData<Vector3>>();

    for(Face f : mesh.faces()){
        Vector3 normal;
        for(Vertex v : f.adjacentVertices()){
            //normal += ;
        }
    }

    return normals;
}