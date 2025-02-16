#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

// #include "args/args.hxx"
#include "imgui.h"

#include "cloth_simulator.hpp"

// test wind.

// Some algorithm parameters
float param1 = 42.0;

// Example computation function -- this one computes and registers a scalar
// quantity
void doWork() {
  polyscope::warning("Computing Gaussian curvature.\nalso, parameter value = " +
                     std::to_string(param1));
}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {

  if (ImGui::Button("do work")) {
    doWork();
  }

  ImGui::SliderFloat("param", &param1, 0., 100.);
}

std::tuple<MatrixXr, MatrixXi> create_rectangle_geometry(
    const Vector3r& origin, double dy, double dz, const std::array<int, 2>& cell_num) {

    int cy = cell_num[0], cz = cell_num[1];
    int ny = cy + 1, nz = cz + 1;

    // Create vertices
    int num_vertices = ny * nz;
    MatrixXr vertices(3, num_vertices);

    // Fill vertices for the main cuboid
    int v_index = 0;
    for (int k = 0; k < nz; ++k) {
        for (int j = 0; j < ny; ++j) {
            v_index = k * ny + j;
            vertices.col(v_index) = origin + Vector3r(0.0, j * dy, k * dz);
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

int main() {
    // Parameters.
    double dt = 1e-2;
    double rho = 1.0;
    double stiffness_stretch = 1.0;
    double stiffness_bending = 0.2;
    double damping = 0.1;
    int solver_iteration = 4;
    Vector3r g {0.0, 0.0, -10.0};
    int frame_num = 2;

    // A piece of cloth.
    Vector3r origin(0.0, 0.0, 0.5);
    std::array<int, 2> cell_num = {10, 10};
    double dy = 1.0 / cell_num[0], dz = 1.0 / cell_num[1];
    auto [vertices, surfaces] = create_rectangle_geometry(origin, dy, dz, cell_num);
    int num_vertices = vertices.cols();
    std::cout << "Initialize geometry done." << std::endl;

    // Create a cloth simulator.
    ClothSimulator sim;
    sim.add_mesh(vertices, surfaces, rho);
    for (int i = 0; i < 3; ++i) {
        sim.add_dirichlet(0, i, vertices(i, 0));
        sim.add_dirichlet((cell_num[0] + 1) * cell_num[1], i, vertices(i, (cell_num[0] + 1) * cell_num[1]));
    }
    sim.add_constraint(0, stiffness_stretch);
    sim.add_constraint(1, stiffness_bending);
    sim.init();

    sim.set_force(g);
    Eigen::Matrix<double, 3, Eigen::Dynamic> wind = MatrixXr::Zero(3, num_vertices);
    double angle = M_PI / 3;
    wind.row(0) = VectorXr::Ones(num_vertices) * std::cos(angle);
    wind.row(1) = VectorXr::Ones(num_vertices) * std::sin(angle);
    sim.set_force(wind);
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
    polyscope::view::lookAt(glm::vec3{2., 0.5, 1.}, glm::vec3{0.0, 0.5, 1.0});

    // set the ground location manually
    polyscope::options::groundPlaneHeightMode = polyscope::GroundPlaneHeightMode::Manual;
    polyscope::options::groundPlaneHeight = 0.; // in world coordinates along the up axis

    // set soft shadows on the ground
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::Tile;
    polyscope::view::upDir = polyscope::UpDir::ZUp;                 // set +Z as up direction
    polyscope::options::groundPlaneHeightFactor = -0.1;  // adjust the plane height
    polyscope::options::shadowDarkness = 0.1;            // lighter shadows

    // Set the callback function
    polyscope::state::userCallback = myCallback;

    auto psMesh = polyscope::registerSurfaceMesh("input mesh", vertices.transpose(), surfaces.transpose());
    std::cout << "Initialize rendering done." << std::endl;

    /////////////////////////
    // Begin simulation.
    /////////////////////////
    std::cout << "Begin simulation." << std::endl;
    while (!polyscope::windowRequestsClose()) {
        sim.forward(dt, opt);

        psMesh->updateVertexPositions(sim.position_.transpose());
        polyscope::frameTick(); 
    }

    return 0;
}