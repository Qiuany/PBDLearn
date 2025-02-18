#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

// #include "args/args.hxx"
#include "imgui.h"

#include "cloth_simulator.hpp"

// test wind.

std::tuple<MatrixXr, MatrixXi> create_rectangle_geometry(
    const Vector3r& origin, const Matrix3r R, double dy, double dz, const std::array<int, 2>& cell_num) {

    int cy = cell_num[0], cz = cell_num[1];
    int ny = cy + 1, nz = cz + 1;
    double half_ly = cy * dy / 2., half_lz = cz * dz / 2.;

    // Create vertices
    int num_vertices = ny * nz;
    MatrixXr vertices(3, num_vertices);

    // Fill vertices for the main cuboid
    int v_index = 0;
    for (int k = 0; k < nz; ++k) {
        for (int j = 0; j < ny; ++j) {
            v_index = k * ny + j;
            vertices.col(v_index) = origin + R * Vector3r(0.0, j * dy - half_ly, k * dz - half_lz);
        }
    }

    // Create elements
    int num_elements = 2 * (cy * cz);
    MatrixXi elements(3, num_elements);
    int e_index = 0;

    for (int y = 0; y < cy; ++y) {
        for (int z = 0; z < cz; ++z) {
            int v1 = z * ny + y;
            int v2 = v1 + 1;
            int v3 = v1 + ny;
            int v4 = v1 + ny + 1;

            elements.col(e_index++) << v1, v2, v4;
            elements.col(e_index++) << v1, v4, v3;
        }
    }

    return {vertices, elements};
}

// Callback function.
int reset = 0;

void doWork() {
    reset = 1;
}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {
    reset = 0;

    if (ImGui::Button("Reset")) {
        doWork();
    }
}

int main() {
    // Parameters.
    double dt = 0.01;
    double rho = 1.0;
    double stiffness_stretch = 1.0;
    double stiffness_bending = 0.05;
    double stiffness_collision = 1.0;
    double plane_restitution = 0.5;
    double plane_thickness = 0.01;
    Vector3r plane_normal {0.0, 0.0, 1.0};
    Vector3r plane_point {0.0, 0.0, 0.0};
    double sphere_radius = 0.2;
    double sphere_restitution = 0.5;
    double sphere_thickness = 0.01;
    Vector3r sphere_center {0.0, 0.0, 0.5};
    Vector3r cloth_center {0.0, 0.0, 0.8};

    double damping = 0.1;
    int solver_iteration = 4;
    Vector3r g {0.0, 0.0, -10.0};
    int frame_num = 2;

    // A piece of cloth.
    std::array<int, 2> cell_num = {10, 10};
    double dy = 1.0 / cell_num[0], dz = 1.0 / cell_num[1];
    Vector3r axis = Vector3r::UnitY();
    double rotate = M_PI / 2;
    Matrix3r R = Eigen::AngleAxisd(rotate, axis).toRotationMatrix();
    auto [vertices, surfaces] = create_rectangle_geometry(cloth_center, R, dy, dz, cell_num);
    int num_vertices = vertices.cols();
    std::cout << "Initialize geometry done." << std::endl;

    // Create a cloth simulator.
    ClothSimulator sim;
    sim.add_mesh(vertices, surfaces, rho);
    sim.add_constraint(0, stiffness_stretch);
    sim.add_constraint(1, stiffness_bending);
    sim.add_constraint(2, stiffness_collision);
    sim.add_plane(plane_normal, plane_point, plane_thickness, plane_restitution);
    sim.add_sphere(sphere_center, sphere_radius, sphere_thickness, sphere_restitution);
    sim.init();

    sim.set_force(g);
    std::cout << "Initialize simulator done." << std::endl;

    Options opt;
    opt.real_option()["damping"] = damping;
    opt.integer_option()["solver_iteration"] = solver_iteration;

    /////////////////////////
    // Initialize polyscope.
    /////////////////////////
    // a few camera options
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);
    polyscope::view::setFrontDir(polyscope::FrontDir::NegYFront);
    polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Free);

    // initialize
    polyscope::init();

    // set the camera pose explicitly
    polyscope::view::lookAt(glm::vec3{2.0, 0.0, 0.6}, glm::vec3{0.0, 0.0, 0.4});

    // set the ground location manually
    polyscope::options::groundPlaneHeightMode = polyscope::GroundPlaneHeightMode::Manual;
    polyscope::options::groundPlaneHeight = 0.; // in world coordinates along the up axis

    // set soft shadows on the ground
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::TileReflection;
    polyscope::view::upDir = polyscope::UpDir::ZUp;                 // set +Z as up direction
    polyscope::options::groundPlaneHeightFactor = -0.1;  // adjust the plane height
    polyscope::options::shadowDarkness = 0.1;            // lighter shadows

    auto psMesh = polyscope::registerSurfaceMesh("input mesh", vertices.transpose(), surfaces.transpose());
    Eigen::Matrix<double, 1,3> points;
    points << sphere_center(0), sphere_center(1), sphere_center(2);
    auto psPoints = polyscope::registerPointCloud("Dirichlet points", points);
    // set some options
    psPoints->setPointRadius(sphere_radius / 2.);
    psPoints->setPointRenderMode(polyscope::PointRenderMode::Sphere);
    std::cout << "Initialize rendering done." << std::endl;

    // Set the callback function
    polyscope::state::userCallback = myCallback;

    /////////////////////////
    // Begin simulation.
    /////////////////////////
    std::cout << "Begin simulation." << std::endl;
    while (!polyscope::windowRequestsClose()) {
        if (reset) {
            sim.reset();
        }

        sim.forward(dt, opt);

        psMesh->updateVertexPositions(sim.position_.transpose());
        polyscope::frameTick(); 
    }

    return 0;
}