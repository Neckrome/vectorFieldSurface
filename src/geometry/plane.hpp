#ifndef DEF_PLANE
#define DEF_PLANE
 
#include <iostream>
#include <string>
#include <math.h>
#include <functional>
#include "geometrycentral/surface/vertex_position_geometry.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

 
class Plane
{
    public:
 
    Plane(Vector3 _n);
    Vector3 projection(Vector3 p);
    
    Vector3 n;

};
 
#endif