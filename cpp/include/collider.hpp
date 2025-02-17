#ifndef PBD_COLLIDER_HPP
#define PBD_COLLIDER_HPP

#include "config.hpp"

struct CollisionConstraintInfo{
    int vIndex;
    double thickness;
    Vector3r concatPosition;
    Vector3r normal;
    Vector3r velocity;
};

class Collider{
    public:
        bool fixed_;
        double restitution_;
        virtual ~Collider() = default;
        virtual bool interset(const Vector3r& p, const Vector3r& p_next, const Vector3r& v, CollisionConstraintInfo& info) const = 0;
};


#endif // PBD_COLLIDER_HPP