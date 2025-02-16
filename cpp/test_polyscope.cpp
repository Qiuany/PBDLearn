#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

// #include "args/args.hxx"
#include "imgui.h"

#include "config.hpp"

// test polyscope.

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

int main() {
    double dt = 1e-2;

    // A cube.
    MatrixXr vertices {{0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0},
                       {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0}, 
                       {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}};
    MatrixXi solids {{0, 0, 0, 3, 0},
                       {2, 1, 4, 5, 3}, 
                       {3, 3, 5, 6, 5}, 
                       {6, 5, 6, 7, 6}};
    MatrixXi surfaces {{0, 0, 4, 5, 1, 3, 0, 0, 0, 0, 2, 3}, 
                       {3, 2, 5, 7, 3, 7, 4, 6, 5, 1, 6, 6}, 
                       {1, 3, 6, 6, 5, 5, 6, 2, 4, 5, 3, 7}};
    int num_vertices = vertices.cols();
    vertices.row(2) += VectorXr::Ones(num_vertices) * 3.0;

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
    polyscope::view::lookAt(glm::vec3{5., 5., 2.}, glm::vec3{0.5, 0.5, 2.});

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


    /////////////////////////
    // Begin simulation.
    /////////////////////////
    Vector3r g {0.0, 0.0, -10.0};
    Vector3r xc = vertices.rowwise().mean();
    Vector3r vc {0.0, 0.0, 0.0};

    while (!polyscope::windowRequestsClose()) {
        vc += g * dt;
        xc += vc * dt;
        for (int i = 0; i < num_vertices; ++i) {
            vertices.col(i) += vc * dt;
        }

        if (xc(2) <= 0.5) {
            vc(2) = -vc(2);
            vertices.row(2) += VectorXr::Ones(num_vertices) * (0.5 - xc(2));
            xc(2) = 0.5;
        }

        psMesh->updateVertexPositions(vertices.transpose());
        polyscope::frameTick(); 
    }

    return 0;
}