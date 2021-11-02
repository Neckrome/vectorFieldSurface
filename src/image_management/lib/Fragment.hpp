

#ifndef FRAGMENT_HPP
#define FRAGMENT_HPP


namespace cpe
{

struct Fragment
{
    float x, y;  // coordonnees en pixel
    float u, v, w;  // coordonnees barycentriques dans le triangle abc
    Fragment(float _x, float _y, float _u, float _v, float _w):
            x(_x), y(_y), u(_u), v(_v), w(_w){};
};


}

#endif
