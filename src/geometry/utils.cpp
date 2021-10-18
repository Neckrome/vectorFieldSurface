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
    Creates a bunch of points on a plane
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

std::tuple<std::unique_ptr<std::vector<Vector3>>, std::unique_ptr<std::vector<Vector3>>> Utils::getProjectedNormals(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry){
    return Utils::getProjectedNormals(mesh, geometry, Plane(Vector3{0.0f, 1.0f, 0.0f}));
}

std::tuple<std::unique_ptr<std::vector<Vector3>>, std::unique_ptr<std::vector<Vector3>>> Utils::getProjectedNormals(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, Plane plane){
    /**
    TODO
    */
    std::unique_ptr<std::vector<Vector3>> pts = std::make_unique<std::vector<Vector3>>(); pts->resize(mesh.nFaces());
    std::unique_ptr<std::vector<Vector3>> normals = std::make_unique<std::vector<Vector3>>(); normals->resize(mesh.nFaces());

    int i = 0;
    for(Face f : mesh.faces()){
        Vector3 p0 = geometry.inputVertexPositions[f.halfedge().vertex()];
        Vector3 p1 = geometry.inputVertexPositions[f.halfedge().next().vertex()];
        Vector3 p2 = geometry.inputVertexPositions[f.halfedge().next().next().vertex()];
        Vector3 normal = cross(p1-p0, p2-p1); normal /= norm(normal);
        
        (*normals)[i] = plane.projection(normal);
        (*pts)[i++] = plane.projection((p0 + p1 + p2)/3);
    }

    for(auto elem : *normals){
        std::cout << elem << std::endl;
    }

    return std::tuple<std::unique_ptr<std::vector<Vector3>>, std::unique_ptr<std::vector<Vector3>>>(std::move(pts), std::move(normals));
}

std::unique_ptr<std::vector<Vector3>> Utils::getNormals(std::vector<Vector3>& projectedNormals){
    return Utils::getNormals(projectedNormals, Plane(Vector3{0.0f, 1.0f, 0.0f}));
}

std::unique_ptr<std::vector<Vector3>> Utils::getNormals(std::vector<Vector3>& projectedNormals, Plane plane){
    /**
    TODO
    */
    std::unique_ptr<std::vector<Vector3>> normals = std::make_unique<std::vector<Vector3>>(); normals->resize(projectedNormals.size());
    int i = 0;
    for(auto p_n : projectedNormals){
        (*normals)[i++] = p_n + sqrt(1 - norm(p_n)*norm(p_n)) * plane.n;
    }

    return normals;
}