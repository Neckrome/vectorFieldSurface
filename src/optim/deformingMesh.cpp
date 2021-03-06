#include "deformingMesh.hpp"
#include <vector>

#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include "geometrycentral/numerical/linear_solvers.h"

#include <Eigen/Sparse>

using namespace geometrycentral;
using namespace geometrycentral::surface;

DeformingMesh::DeformingMesh() {}

std::unique_ptr<VertexPositionGeometry> DeformingMesh::iterativeSolve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, VertexPositionGeometry& origGeom, VertexData<Vector3>& bondaryFixedValues, FaceData<Vector3>& normals, VertexData<Vector3>& debugGradient, float lr, float nw){
    /**
    TODO
    */
    /*
    std::unique_ptr<VertexPositionGeometry> newGeometry = geometry.copy();
    debugGradient.fill(Vector3{0.0f, 0.0f, 0.0f});

    for(int iter = 0; iter < 10000; ++iter){
        std::unique_ptr<VertexPositionGeometry> tmpGeometry = newGeometry->copy();
        Vector3 sumGrad = Vector3{0.0f, 0.0f, 0.0f};
        float alignement = 0.0f;
        for(Face f : mesh.faces()){
            Vector3 n = normals[f];
            for(Vertex v : f.adjacentVertices()){

                Vector3 P = newGeometry->inputVertexPositions[v]; 
                Vector3 P1 = newGeometry->inputVertexPositions[v.halfedge().next().vertex()];
                Vector3 P2 = newGeometry->inputVertexPositions[v.halfedge().next().next().vertex()];
                Vector3 origPoint = origGeom.inputVertexPositions[v];

                Vector3 grad_pfij_Em = 2*dot(2*P - (P1 + P2), n)*n; grad_pfij_Em[0] = 0.0f; grad_pfij_Em[2] = 0.0f;
                Vector3 grad_p_Ed = 2*(P - origPoint); grad_p_Ed[1] = 0.0f;
                Vector3 grad_p_E = nw*grad_pfij_Em + grad_p_Ed;
                sumGrad = sumGrad + grad_p_E;
                alignement += dot(n, cross(P1-P, P2-P1));
                debugGradient[v] = debugGradient[v] + grad_p_E;

                tmpGeometry->inputVertexPositions[v] -= lr * grad_p_E;

                if(bondaryFixedValues[v] != Vector3{0.0f, 0.0f, 0.0f}){
                    tmpGeometry->inputVertexPositions[v] = bondaryFixedValues[v];
                }
            }
        }
        std::cout << alignement << std::endl;
        newGeometry = tmpGeometry->copy();
    }

    return newGeometry;*/

    std::unique_ptr<VertexPositionGeometry> newGeometry = geometry.copy();
    debugGradient.fill(Vector3{0.0f, 0.0f, 0.0f});

    for(int iter = 0; iter < 10000; ++iter){
        std::unique_ptr<VertexPositionGeometry> tmpGeometry = newGeometry->copy();

        for(Vertex v : mesh.vertices()){
            for(Face f : v.adjacentFaces()){
                
            }
        }
    }


    for(int iter = 0; iter < 10000; ++iter){
        std::unique_ptr<VertexPositionGeometry> tmpGeometry = newGeometry->copy();
        Vector3 sumGrad = Vector3{0.0f, 0.0f, 0.0f};
        float alignement = 0.0f;
        for(Face f : mesh.faces()){
            Vector3 n = normals[f];
            for(Vertex v : f.adjacentVertices()){

                Vector3 P = newGeometry->inputVertexPositions[v]; 
                Vector3 P1 = newGeometry->inputVertexPositions[v.halfedge().next().vertex()];
                Vector3 P2 = newGeometry->inputVertexPositions[v.halfedge().next().next().vertex()];
                Vector3 origPoint = origGeom.inputVertexPositions[v];

                Vector3 grad_pfij_Em = dot(P - P2, n)*n + dot(P - P1, n)*n; //grad_pfij_Em[0] = 0.0f; grad_pfij_Em[2] = 0.0f;
                Vector3 grad_p_Ed = 2*(P - origPoint); grad_p_Ed[1] = 0.0f;
                Vector3 grad_p_E = nw*grad_pfij_Em + grad_p_Ed;
                sumGrad = sumGrad + grad_p_E;
                alignement += dot(n, cross(P1-P, P2-P1));
                debugGradient[v] = debugGradient[v] + grad_p_E;

                tmpGeometry->inputVertexPositions[v] -= lr * grad_p_E;

                if(bondaryFixedValues[v] != Vector3{0.0f, 0.0f, 0.0f}){
                    //tmpGeometry->inputVertexPositions[v] = bondaryFixedValues[v];
                }
            }
        }
        //std::cout << alignement << std::endl;
        newGeometry = tmpGeometry->copy();
    }

    return newGeometry; 

}


std::unique_ptr<VertexPositionGeometry> DeformingMesh::iterativeSolve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, VertexPositionGeometry& origGeom, FaceData<Vector3>& normals, VertexData<Vector3>& debugGradient, float lr, float nw){
    /**
    TODO
    */
    
    std::unique_ptr<VertexPositionGeometry> newGeometry = geometry.copy();
    debugGradient.fill(Vector3{0.0f, 0.0f, 0.0f});

    for(int iter = 0; iter < 3; ++iter){
        std::unique_ptr<VertexPositionGeometry> tmpGeometry = newGeometry->copy();
        Vector3 sumGrad = Vector3{0.0f, 0.0f, 0.0f};
        float alignement = 0.0f;
        for(Face f : mesh.faces()){
            Vector3 n = normals[f];
            for(Vertex v : f.adjacentVertices()){

                Vector3 P = newGeometry->inputVertexPositions[v]; 
                Vector3 P1 = newGeometry->inputVertexPositions[v.halfedge().next().vertex()];
                Vector3 P2 = newGeometry->inputVertexPositions[v.halfedge().next().next().vertex()];
                Vector3 origPoint = origGeom.inputVertexPositions[v];

                Vector3 grad_pfij_Em = 2*dot(2*P - (P1 + P2), n)*n; 
                //Vector3 grad_pfij_Em = 2*( 1-dot(cross(P1-P, P2-P1)/norm(cross(P1-P, P2-P1)), n) )*n; 
                Vector3 grad_p_Ed = 2*(P - origPoint); //grad_p_Ed[1] = 0.0f;
                Vector3 grad_p_E = nw*grad_pfij_Em + grad_p_Ed;
                sumGrad = sumGrad + grad_p_E;
                alignement += dot(n, cross(P1-P, P2-P1)/norm(cross(P1-P, P2-P1)));
                debugGradient[v] = debugGradient[v] + grad_p_E;

                tmpGeometry->inputVertexPositions[v] -= lr * grad_p_E;

            }
        }
        //std::cout << alignement << std::endl;
        newGeometry = tmpGeometry->copy();
    }

    return newGeometry;
}

void fillVector(Eigen::VectorXd& v, VertexPositionGeometry& geometry){
    for(size_t i = 0; i < v.size(); ++i){

    }
}

std::unique_ptr<VertexPositionGeometry> DeformingMesh::analyticSolve(ManifoldSurfaceMesh& mesh, VertexPositionGeometry& geometry, VertexData<Vector3>& bondaryFixedValues, FaceData<Vector3>& normals){
    /**TODO
     * /
    */
    size_t nFaces = mesh.nFaces();
    size_t nVertices = mesh.nVertices();
    Eigen::SparseMatrix<double> C(3*nFaces, nVertices);
    Eigen::SparseMatrix<double> Id(3*nFaces, nVertices);
    Id.setIdentity();

    Eigen::VectorXd q = Eigen::VectorXd::Zero(nVertices);

    for(size_t fId = 0; fId < nFaces; ++fId){
        Face f = mesh.face(fId);
        Vector3 n = normals[f];
        Vertex v = f.halfedge();
        size_t v0Id = v.getindex();
        size_t v1Id = v.halfedge().next().vertex().getindex();
        size_t v2Id = v.halfedge().next().next().vertex().getindex();
        Vector3 P0 = geometry->inputVertexPositions[v]; 
        Vector3 P1 = geometry->inputVertexPositions[v.halfedge().next().vertex()];
        Vector3 P2 = geometry->inputVertexPositions[v.halfedge().next().next().vertex()];
        C.coeffRef(3*fId  , vId) = ; C.coeffRef(3*fId  , vId+1) = ; C.coeffRef(3*fId  , vId+2) = ;
        C.coeffRef(3*fId+1, vId) = ; C.coeffRef(3*fId+1, vId+1) = ; C.coeffRef(3*fId+1, vId+2) = ; 
        C.coeffRef(3*fId+2, vId) = ; C.coeffRef(3*fId+2, vId+1) = ; C.coeffRef(3*fId+2, vId+2) = ;
        
    }

}






