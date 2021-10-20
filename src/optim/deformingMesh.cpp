#include "deformingMesh.hpp"
#include <vector>

#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include "geometrycentral/numerical/linear_solvers.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

DeformingMesh::DeformingMesh() {}

std::unique_ptr<VertexPositionGeometry> DeformingMesh::iterativeSolve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, VertexPositionGeometry& origGeom, FaceData<Vector3>& normals){
    /**
    TODO
    */
    
    std::unique_ptr<VertexPositionGeometry> newGeometry = geometry.copy();
    float lr = 0.04f;

    for(int iter; iter < 10; ++iter){
        std::unique_ptr<VertexPositionGeometry> tmpGeometry = newGeometry->copy();
        for(Face f : mesh.faces()){
            Vector3 n = normals[f];
            
            for(Vertex v : f.adjacentVertices()){
                size_t nAdjacentVertices = v.degree();
                Vector3 P = newGeometry->inputVertexPositions[v]; 
                Vector3 P1 = newGeometry->inputVertexPositions[v.halfedge().next().vertex()];
                Vector3 P2 = newGeometry->inputVertexPositions[v.halfedge().next().next().vertex()];
                Vector3 origPoint = origGeom.inputVertexPositions[v];

                Vector3 grad_pfij_Em = 2*dot(2*P - (P1 + P2), n)*n;  //grad_pfij_Em[0] = 0.0f; grad_pfij_Em[2] = 0.0f;
                Vector3 grad_p_Ed = 2*(P - origPoint); grad_p_Ed[1] = 0;
                Vector3 grad_p_E = grad_pfij_Em + grad_p_Ed;

                tmpGeometry->inputVertexPositions[v] -= lr * grad_p_E / nAdjacentVertices;
            }
        }
        newGeometry = tmpGeometry->copy();
    }

    return newGeometry;
}

std::unique_ptr<VertexPositionGeometry> DeformingMesh::analyticSolve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, FaceData<Vector3>& normals){

}


