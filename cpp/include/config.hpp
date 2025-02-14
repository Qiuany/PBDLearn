#ifndef PBD_CONFIG_H
#define PBD_CONFIG_H

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Sparse"

#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <cmath>
#include <set>
#include <memory>
#include <chrono>
#include <omp.h>

using Vector2r = Eigen::Matrix<double, 2, 1>;
using Vector3r = Eigen::Matrix<double, 3, 1>;
using Vector4r = Eigen::Matrix<double, 4, 1>;
using Vector6r = Eigen::Matrix<double, 6, 1>;
using Vector8r = Eigen::Matrix<double, 8, 1>;
using Vector9r = Eigen::Matrix<double, 9, 1>;
using VectorXr = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Vector3i = Eigen::Matrix<int, 3, 1>;
using VectorXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;

using RowVectorXr = Eigen::Matrix<double, 1, Eigen::Dynamic>;

using Matrix2r = Eigen::Matrix<double, 2, 2>;
using Matrix3r = Eigen::Matrix<double, 3, 3>;
using Matrix4r = Eigen::Matrix<double, 4, 4>;
using Matrix6r = Eigen::Matrix<double, 6, 6>;
using Matrix8r = Eigen::Matrix<double, 8, 8>;
using Matrix9r = Eigen::Matrix<double, 9, 9>;
using Matrix12r = Eigen::Matrix<double, 12, 12>;
using Matrix23 = Eigen::Matrix<double, 2, 3>;
using Matrix32 = Eigen::Matrix<double, 3, 2>;
using Matrix39 = Eigen::Matrix<double, 3, 9>;
using Matrix3_12 = Eigen::Matrix<double, 3, 12>;
using Matrix43 = Eigen::Matrix<double, 4, 3>;
using Matrix49 = Eigen::Matrix<double, 4, 9>;
using Matrix64 = Eigen::Matrix<double, 6, 4>;
using Matrix9_27 = Eigen::Matrix<double, 9, 27>;
using Matrix12_9 = Eigen::Matrix<double, 12, 9>;
using Matrix27_9 = Eigen::Matrix<double, 27, 9>;
using MatrixXr = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;

using SparseMatrixXr = Eigen::SparseMatrix<double>;

static const std::string RedHead() {
    return "\x1b[6;30;91m";
}

static const std::string RedTail() {
    return "\x1b[0m";
}

inline void PrintError(const std::string& location, const std::string& message) {
    std::stringstream ss;
    ss << RedHead() << "[" << location << "]: " << message << RedTail() << std::endl;
    std::cerr << ss.str();
    throw std::runtime_error(ss.str());
}

inline void CheckCondition(const bool condition, const std::string& location, const std::string& message) {
    if (!condition) {
        PrintError(location, message);
    }
}

#endif