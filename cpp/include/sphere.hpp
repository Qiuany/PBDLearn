#ifndef PBD_SPHERE_HPP
#define PBD_SPHERE_HPP

#include "config.hpp"
#include "collider.hpp"

class Sphere : public Collider{
    public:
        Vector3r point_;
        double radius_;
        double thickness_;
        
        Sphere(const Vector3r& point, double radius, double thickness, double restitution)
                : point_(point), radius_(radius), thickness_(thickness) {
            fixed_ = true;
            if (restitution < 0.0 || restitution > 1.0) {
                std::cerr << "Restitution should be in [0, 1]." << std::endl;
                exit(1);
            }
            restitution_ = restitution;
        }

        ~Sphere() {};
        bool interset(const Vector3r& p, const Vector3r& p_next, const Vector3r& v,
                      CollisionConstraintInfo& info) override {
            double dist = (p_next - point_).norm() - radius_;
            if (dist < thickness_){
                // We choose the contact point as the nearest point to p_next on the sphere surface.
                info.thickness = thickness_;
                info.normal = (p_next - point_) / (p_next - point_).norm();
                info.concatPosition = point_ + radius_ * info.normal;
                // v0' = (v0(m0 - em1) + (1 + e)m1v1) / (m0 + m1).
                // v1' = (v1(m1 - em0) + (1 + e)m0v0) / (m0 + m1).
                Vector3r v1_normal = v.dot(info.normal) * info.normal;
                Vector3r v1_normal_new =  - restitution_ * v1_normal;
                info.velocity = v - v1_normal + v1_normal_new;
                return true;
            }
            return false;
        }
};

#endif // PBD_PLANE_HPP