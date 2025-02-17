#ifndef PBD_PLANE_HPP
#define PBD_PLANE_HPP

#include "config.hpp"
#include "collider.hpp"

class Plane : public Collider{
    public:
        Vector3r normal_;
        Vector3r point_;
        double thickness_;
        
        Plane(const Vector3r& normal, const Vector3r& point, double thickness, double restitution)
         : normal_(normal), point_(point), thickness_(thickness) {
            fixed_ = true;
            if (restitution < 0.0 || restitution > 1.0) {
                std::cerr << "Restitution should be in [0, 1]." << std::endl;
                exit(1);
            }
            restitution_ = restitution;
        }

        ~Plane() {};
        bool interset(const Vector3r& p, const Vector3r& p_next, const Vector3r& v, CollisionConstraintInfo& info) const override {
            double dist = normal_.dot(p_next - point_);
            if (dist < thickness_) {
                info.thickness = thickness_;
                info.concatPosition = p_next - dist * normal_;
                info.normal = normal_;
                info.velocity = v - v.dot(normal_) * normal_ * (restitution_ + 1);  // v.dot(normal_) < 0.
                return true;
            }
            return false;
        }
};

#endif // PBD_PLANE_HPP