#ifndef PBD_CLOTH_SIMULATOR_HPP
#define PBD_CLOTH_SIMULATOR_HPP

#include "config.hpp"
#include "options.hpp"
#include "SparseMatrixUtils.hpp"

struct DistanceConstraintInfo{
    double restLength;
    int vIndex0;
    int vIndex1;
};

struct BendingConstraintInfo{
    double restAngle;
    int vIndex0;
    int vIndex1;
    int vIndex2;
    int vIndex3;
};

class ClothSimulator
{    
    public:
        int vNum, triNum;
        const int v_dim = 3;
        const int e_dim = 3;

        double trirho;
        Eigen::Matrix<double, 3, Eigen::Dynamic> rest_position_;
        Eigen::Matrix<double, 3, Eigen::Dynamic> normal_;
        Eigen::Matrix<int, 3, Eigen::Dynamic> faces_;
        std::vector<double> mass_vec_;
        std::vector<double> mass_inv_vec_;
        SparseMatrixXr mass_inv_matrix_;

        // dynamics.
        Eigen::Matrix<double, 3, Eigen::Dynamic> position_;
        Eigen::Matrix<double, 3, Eigen::Dynamic> next_position_;
        Eigen::Matrix<double, 3, Eigen::Dynamic> velocity_;
        Eigen::Matrix<double, 3, Eigen::Dynamic> next_velocity_;
        Eigen::Matrix<double, 3, Eigen::Dynamic> force_;
        Eigen::Matrix<double, 3, Eigen::Dynamic> external_acceleration_;
        Vector3r a_;
        // Dirichlet.
        std::vector<std::pair<int, double>> dirichlet_;
        // Constraints.
        std::map<int, double> stiffness_;
        std::vector<DistanceConstraintInfo> distance_constraints_;
        std::vector<BendingConstraintInfo> bending_constraints_;
        
    public:
        ClothSimulator();
        ~ClothSimulator() {};
        void add_mesh(
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& vertices, 
            const Eigen::Matrix<int, 3, Eigen::Dynamic>& surfaces,
            const double trimeshrho
        );
        void add_constraint(const int constraint_type, const double stiffness);
        void add_dirichlet(const int dof_index, const int dim_index, const double value);
        void set_force(const Vector3r& a) { a_ = a; }
        void set_force(const Eigen::Matrix<double, 3, Eigen::Dynamic>& force) {force_ = force;}
        void init();
        void build_mass_normal();
        void build_topology();
        void forward(const double _dt, const Options& opt);
        void compute_normal();
        double compute_stiffness(const int constraint_type, const int solver_iteration);
        void project_distance_constraint(const double _dt, const int solver_iteration);
        void project_bending_constraint(const double _dt, const int solver_iteration);
        void project_dirichlet();
};

const double compute_dihedral_angle(const Vector3r& p0, const Vector3r& p1, const Vector3r& p2, const Vector3r& p3);

#endif // PBD_CLOTH_SIMULATOR_HPP