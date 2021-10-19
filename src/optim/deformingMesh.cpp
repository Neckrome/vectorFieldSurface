#include "deformingMesh.hpp"
#include <vector>
#include <array>
#include <functional>
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_mesh_factories.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

DeformingMesh::DeformingMesh() {}

std::unique_ptr<VertexPositionGeometry> DeformingMesh::solve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, FaceData<Vector3>& normals){
    /**
    TODO
    */
    
    std::unique_ptr<VertexPositionGeometry> newGeometry = geometry.copy();
    float lr = 0.001f;

    for(int iter; iter < 10000; ++iter){
        std::unique_ptr<VertexPositionGeometry> tmpGeometry = newGeometry->copy();
        for(Face f : mesh.faces()){
            Vector3 n = normals[f];
            
            for(Vertex v : f.adjacentVertices()){
                Vector3 P = newGeometry->inputVertexPositions[v]; 
                Vector3 P1 = newGeometry->inputVertexPositions[v.halfedge().next().vertex()];
                Vector3 P2 = newGeometry->inputVertexPositions[v.halfedge().next().next().vertex()];
                Vector3 origPoint = geometry.inputVertexPositions[v];

                Vector3 grad_pfij_Em = 2*dot(2*P - (P1 + P2), n)*n;  //grad_pfij_Em[0] = 0.0f; grad_pfij_Em[2] = 0.0f;
                Vector3 grad_p_Ed = 2*(P - origPoint); grad_p_Ed[1] = 0;
                Vector3 grad_p_E = grad_pfij_Em + 0.01*grad_p_Ed; //grad_p_E[0] = 0.0f; grad_p_E[2] = 0.0f;

                tmpGeometry->inputVertexPositions[v] -= lr * grad_p_E;
            }
        }
        newGeometry = tmpGeometry->copy();
    }

    return newGeometry;
}


