#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

// #include "args/args.hxx"
#include "imgui.h"

#include "cloth_simulator.hpp"

// test wind.

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

// Callback function.
float param_angle_z = 30.;
float param_angle_xy = 60.;
float param_magnitude = 1.0;
int apply_wind = 0;

void doWork() {
    apply_wind = 1;
}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {
    apply_wind = 0;
    ImGui::SliderFloat("angle_xy", &param_angle_xy, 0., 360.);
    ImGui::SliderFloat("angle_z", &param_angle_z, -90., 90.);
    ImGui::SliderFloat("magnitude", &param_magnitude, 0., 5.);

    if (ImGui::Button("Apply wind")) {
        doWork();
    }
}

int main() {
    // Parameters.
    double dt = 1e-2;
    double rho = 1.0;
    double stiffness_stretch = 1.0;
    double stiffness_bending = 0.2;
    double stiffness_collision = 1.0;
    double restitution_plane = 0.5;
    double thickness_plane = 0.02;
    double damping = 0.1;
    int solver_iteration = 4;
    Vector3r g {0.0, 0.0, -10.0};
    int frame_num = 2;

    // A piece of cloth.
    Vector3r origin(0.0, 0.0, 0.2);
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
    sim.add_constraint(2, stiffness_collision);
    sim.add_plane(Vector3r(0.0, 0.0, 1.0), Vector3r(0.0, 0.0, 0.0), thickness_plane, restitution_plane);
    sim.init();

    sim.set_force(g);
    double angle_z = M_PI / 6;
    double angle_xy = M_PI / 3;
    double magnitude = 1.0;
    Vector3r wind {std::cos(angle_z) * std::cos(angle_xy), 
                   std::cos(angle_z) * std::sin(angle_xy), 
                   std::sin(angle_z)};
    wind *= magnitude;
    sim.set_wind_force(wind);
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
    polyscope::view::lookAt(glm::vec3{2.5, 0.5, 1.2}, glm::vec3{0.0, 0.5, 0.7});

    // set the ground location manually
    polyscope::options::groundPlaneHeightMode = polyscope::GroundPlaneHeightMode::Manual;
    polyscope::options::groundPlaneHeight = 0.; // in world coordinates along the up axis

    // set soft shadows on the ground
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::TileReflection;
    polyscope::view::upDir = polyscope::UpDir::ZUp;                 // set +Z as up direction
    polyscope::options::groundPlaneHeightFactor = -0.1;  // adjust the plane height
    polyscope::options::shadowDarkness = 0.1;            // lighter shadows

    auto psMesh = polyscope::registerSurfaceMesh("input mesh", vertices.transpose(), surfaces.transpose());
    Eigen::Matrix<double, 2,3> points;
    points << vertices(0, 0), vertices(1, 0), vertices(2, 0),
              vertices(0, (cell_num[0] + 1) * cell_num[1]), vertices(1, (cell_num[0] + 1) * cell_num[1]), vertices(2, (cell_num[0] + 1) * cell_num[1]);
    auto psPoints = polyscope::registerPointCloud("Dirichlet points", points);
    // set some options
    psPoints->setPointRadius(0.02);
    psPoints->setPointRenderMode(polyscope::PointRenderMode::Sphere);
    std::cout << "Initialize rendering done." << std::endl;

    // Set the callback function
    polyscope::state::userCallback = myCallback;

    /////////////////////////
    // Begin simulation.
    /////////////////////////
    std::cout << "Begin simulation." << std::endl;
    while (!polyscope::windowRequestsClose()) {
        if (apply_wind) {
            angle_z = static_cast<double>(param_angle_z / 180.0 * M_PI);
            angle_xy = static_cast<double>(param_angle_xy / 180.0 * M_PI);
            wind << std::cos(angle_z) * std::cos(angle_xy), 
                    std::cos(angle_z) * std::sin(angle_xy), 
                    std::sin(angle_z);
            wind *= static_cast<double>(param_magnitude);
            sim.set_wind_force(wind);
        }
        sim.forward(dt, opt);

        psMesh->updateVertexPositions(sim.position_.transpose());
        polyscope::frameTick(); 
    }

    return 0;
}