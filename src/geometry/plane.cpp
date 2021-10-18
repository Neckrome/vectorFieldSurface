#include "plane.hpp"
#include <vector>
#include <array>
#include <functional>
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_mesh_factories.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

Plane::Plane(Vector3 _n) : n(_n) {}

Vector3 Plane::projection(Vector3 p){
    return p - dot(n, p) / norm(n) * n;
}