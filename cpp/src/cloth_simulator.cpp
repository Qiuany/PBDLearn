#include "cloth_simulator.hpp"

ClothSimulator::ClothSimulator()
{
    vNum = 0;
    triNum = 0;
    trirho = 0.0;
    stiffness_[0] = -1.0;  // distance constraint.
    stiffness_[1] = -1.0;  // bending constraint.
}

void ClothSimulator::add_mesh(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& vertices, 
    const Eigen::Matrix<int, 3, Eigen::Dynamic>& surfaces,
    const double trimeshrho
) {
    faces_ = surfaces;
    vNum = vertices.cols();
    triNum = surfaces.cols();
    rest_position_ = vertices;
    position_ = vertices;

    trirho = trimeshrho;
}

void ClothSimulator::add_constraint(const int constraint_type, const double stiffness) {
    if (constraint_type >= 0 && constraint_type <= 2)
        stiffness_[constraint_type] = stiffness;
    else
        std::cerr << "Constraint type not supported." << std::endl;
}

void ClothSimulator::add_dirichlet(const int dof_index, const int dim_index, const double value) {
    std::pair<int, double> item = std::make_pair(dim_index + dof_index * 3, value);
    dirichlet_.push_back(item);
}

void ClothSimulator::add_plane(const Vector3r& normal, const Vector3r& point, double thickness, double restitution) {
    colliders_.push_back(std::make_shared<Plane>(normal, point, thickness, restitution));
}

void ClothSimulator::init()
{
    next_position_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, vNum);
    velocity_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, vNum);
    next_velocity_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, vNum);
    force_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, vNum);
    external_acceleration_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, vNum);
    normal_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, vNum);
    a_ = Vector3r::Zero();

    build_mass_normal();
    build_topology();
}

void ClothSimulator::build_mass_normal() {
    // Mass & Normal.
    mass_vec_.resize(vNum);
    for (int i = 0; i < triNum; i++)
    {
        int vidx0 = faces_(0, i);
        int vidx1 = faces_(1, i);
        int vidx2 = faces_(2, i);
        Vector3r x0 = rest_position_.col(vidx0);
        Vector3r x1 = rest_position_.col(vidx1);
        Vector3r x2 = rest_position_.col(vidx2);
        Vector3r x01 = x1 - x0;
        Vector3r x02 = x2 - x0;
        Vector3r n = x01.cross(x02);
        double area = n.norm() / 2.0;
        double mass = trirho * area;
        mass_vec_[vidx0] += mass / 3.0;
        mass_vec_[vidx1] += mass / 3.0;
        mass_vec_[vidx2] += mass / 3.0;
        normal_.col(vidx0) += n;
        normal_.col(vidx1) += n;
        normal_.col(vidx2) += n;
    }
    for (int i = 0; i < vNum; i++)
    {
        normal_.col(i).normalize();
        if (mass_vec_[i] < 1e-6)
            std::cerr << "Mass is too small." << std::endl;
        mass_inv_vec_.push_back(1.0 / mass_vec_[i]);
    }
    mass_inv_matrix_ = FromDiagonal(mass_inv_vec_);
}

void ClothSimulator::build_topology() {
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> edge_map;
    // double avg_area = 1.;
    // if (triNum > 0) avg_area = meshArea_vec[0] / trivIdx_vec[0].size() * 3;
    for (int e = 0; e < triNum; ++e) {
        for (int v = 0; v < 3; ++v) {
            int idx0 = faces_(v, e);
            int idx1 = faces_((v + 1) % 3, e);
            std::pair<int, int> key = (idx0 < idx1) ? std::make_pair(idx0, idx1) : std::make_pair(idx1, idx0);
            if (edge_map.find(key) == edge_map.end()) {
                edge_map[key] = { {e, v} };
                DistanceConstraintInfo info;
                info.vIndex0 = key.first;
                info.vIndex1 = key.second;
                info.restLength = (rest_position_.col(key.first) - rest_position_.col(key.second)).norm();
                distance_constraints_.push_back(info);
            } else {
                // we assume that each edge is shared by at most two triangles when the edge is on the surface.
                int other_e = edge_map[key][0].first;
                int other_v = edge_map[key][0].second;
                edge_map[key].push_back({e, v});
                //  2---1---3
                //    \ | /
                //     \|/
                //      0
                BendingConstraintInfo info;
                info.vIndex0 = key.first;
                info.vIndex1 = key.second;
                if (faces_((v + 1) % 3, e) == key.second) {
                    info.vIndex2 = faces_((v + 2) % 3, e);
                    info.vIndex3 = faces_((other_v + 2) % 3, other_e);
                } else {
                    info.vIndex2 = faces_((other_v + 2) % 3, other_e);
                    info.vIndex3 = faces_((v + 2) % 3, e);
                }
                Vector3r p0 = rest_position_.col(info.vIndex0);
                Vector3r p1 = rest_position_.col(info.vIndex1);
                Vector3r p2 = rest_position_.col(info.vIndex2);
                Vector3r p3 = rest_position_.col(info.vIndex3);
                info.restAngle = compute_dihedral_angle(p0, p1, p2, p3);
                bending_constraints_.push_back(info);
            }
        }
    }
}

void ClothSimulator::forward(const double _dt, const Options& opt) {
    double h = _dt;
    double damping = opt.real_option()["damping"];
    int solver_iteration = opt.integer_option()["solver_iteration"];
    compute_normal();
    // Sympletic Euler.
    for (int i = 0; i < vNum; ++i) {
        double w = mass_inv_vec_[i];
        Vector3r a = a_ + force_.col(i).dot(normal_.col(i)) * normal_.col(i) * w;
        next_velocity_.col(i) = velocity_.col(i) + h * a;
        next_velocity_.col(i) *= std::max(0.0, 1.0 - damping * h * w);
    }
    next_position_ = position_ + h * next_velocity_;

    // Collision detection.
    collision_detection();

    // Project constraints.
    for (int i = 0; i < solver_iteration; ++i) {
        project_distance_constraint(solver_iteration);
        project_bending_constraint(solver_iteration);
        project_collision_constraint(solver_iteration);
        project_dirichlet();
    }

    // Update.
    update_position_velocity(h);
    collision_constraints_.clear();
}

void ClothSimulator::compute_normal() {
    normal_ = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, vNum);
    for (int i = 0; i < triNum; i++)
    {
        int vidx0 = faces_(0, i);
        int vidx1 = faces_(1, i);
        int vidx2 = faces_(2, i);
        Vector3r x0 = position_.col(vidx0);
        Vector3r x1 = position_.col(vidx1);
        Vector3r x2 = position_.col(vidx2);
        Vector3r x01 = x1 - x0;
        Vector3r x02 = x2 - x0;
        Vector3r n = x01.cross(x02);
        normal_.col(vidx0) += n;
        normal_.col(vidx1) += n;
        normal_.col(vidx2) += n;
    }
    for (int i = 0; i < vNum; i++)
    {
        normal_.col(i).normalize();
    }
}

double ClothSimulator::compute_stiffness(const int constraint_type, const int solver_iteration) {
    double power = 1.0 / solver_iteration;
    return 1.0 - std::pow(1.0 - stiffness_[constraint_type], power);
}

void ClothSimulator::project_distance_constraint(const int solver_iteration) {
    double stiffness = compute_stiffness(0, solver_iteration);
    for (int j = 0; j < distance_constraints_.size(); ++j) {
        DistanceConstraintInfo& info = distance_constraints_[j];
        Vector3r p0 = next_position_.col(info.vIndex0);
        Vector3r p1 = next_position_.col(info.vIndex1);
        Vector3r n = p1 - p0;
        double d = n.norm();
        n.normalize();
        double w0 = mass_inv_vec_[info.vIndex0];
        double w1 = mass_inv_vec_[info.vIndex1];
        double w_sum = w0 + w1;
        double diff = d - info.restLength;
        Vector3r correction = stiffness * diff / w_sum * n;
        next_position_.col(info.vIndex0) += w0 * correction;
        next_position_.col(info.vIndex1) -= w1 * correction;
    }
}

void ClothSimulator::project_bending_constraint(const int solver_iteration) {
    double stiffness = compute_stiffness(1, solver_iteration);
    for (int j = 0; j < bending_constraints_.size(); ++j) {
        BendingConstraintInfo& info = bending_constraints_[j];
        double w0 = mass_inv_vec_[info.vIndex0];
        double w1 = mass_inv_vec_[info.vIndex1];
        double w2 = mass_inv_vec_[info.vIndex2];
        double w3 = mass_inv_vec_[info.vIndex3];
        Vector3r p0 = next_position_.col(info.vIndex0);
        Vector3r p1 = next_position_.col(info.vIndex1);
        Vector3r p2 = next_position_.col(info.vIndex2);
        Vector3r p3 = next_position_.col(info.vIndex3);
        Vector3r p01 = p1 - p0;
        Vector3r p02 = p2 - p0;
        Vector3r p03 = p3 - p0;
        Vector3r n1 = p01.cross(p02);
        Vector3r n2 = p01.cross(p03);
        n1.normalize();
        n2.normalize();
        double d = n1.dot(n2);
        double theta = acos(d);
        double diff = theta - info.restAngle;


        Vector3r q3 = (p01.cross(n1) + (n2.cross(p01)) * d) / (p01.cross(p03)).norm();
        Vector3r q2 = (p01.cross(n2) + (n1.cross(p01)) * d) / (p01.cross(p02)).norm();
        Vector3r q1 = - (p02.cross(n2) + (n1.cross(p02)) * d) / (p01.cross(p02)).norm() - (p03.cross(n1) + (n2.cross(p03)) * d) / (p01.cross(p03)).norm();
        Vector3r q0 = -q1 - q2 - q3;

        double scale = stiffness * diff * std::sqrt(1 - d * d);
        double factor = (w0 * q0.squaredNorm() + w1 * q1.squaredNorm() + w2 * q2.squaredNorm() + w3 * q3.squaredNorm());
        if (factor < 1e-6) {
            continue;
        }
        scale /= factor;
        next_position_.col(info.vIndex0) -= w0 * scale * q0;
        next_position_.col(info.vIndex1) -= w1 * scale * q1;
        next_position_.col(info.vIndex2) -= w2 * scale * q2;
        next_position_.col(info.vIndex3) -= w3 * scale * q3;
    }
}

void ClothSimulator::project_dirichlet() {
    for (int i = 0; i < dirichlet_.size(); ++i) {
        int dof_index = dirichlet_[i].first;
        double value = dirichlet_[i].second;
        next_position_.col(dof_index / 3)(dof_index % 3) = value;
        next_velocity_.col(dof_index / 3)(dof_index % 3) = 0.0;
    }
}

const double compute_dihedral_angle(const Vector3r& p0, const Vector3r& p1, const Vector3r& p2, const Vector3r& p3) {
    Vector3r n1 = (p1 - p0).cross(p2 - p0);
    Vector3r n2 = (p1 - p0).cross(p3 - p0);
    n1.normalize();
    n2.normalize();
    double cos_theta = n1.dot(n2);
    return acos(cos_theta);
}

void ClothSimulator::collision_detection() {
    for (int i = 0; i < vNum; ++i) {
        Vector3r p = position_.col(i);
        Vector3r p_next = next_position_.col(i);
        Vector3r v = velocity_.col(i);
        for (int j = 0; j < colliders_.size(); ++j) {
            CollisionConstraintInfo info;
            if (colliders_[j]->interset(p, p_next, v, info)) {
                info.vIndex = i;
                collision_constraints_.push_back(info);
            }
        }
    }
}

void ClothSimulator::project_collision_constraint(const int solver_iteration) {
    double stiffness = compute_stiffness(2, solver_iteration);
    for (int j = 0; j < collision_constraints_.size(); ++j) {
        CollisionConstraintInfo& info = collision_constraints_[j];
        Vector3r p = next_position_.col(info.vIndex);
        double diff = (p - info.concatPosition).dot(info.normal) - info.thickness;
        if (diff < 0.0) {
            Vector3r correction = stiffness * diff * info.normal;
            next_position_.col(info.vIndex) -= correction;
        }
    }
}

void ClothSimulator::update_position_velocity(const double _dt) {
    double h = _dt;
    velocity_ = (next_position_ - position_) / h;
    position_ = next_position_;
    for (int i = 0; i < collision_constraints_.size(); ++i) {
        CollisionConstraintInfo& info = collision_constraints_[i];
        Vector3r new_v = info.velocity;
        velocity_.col(info.vIndex) = new_v;
    }
}