#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>


// Converting vector to so3
Eigen::Matrix<double, 3, 3> VectorToso3(Eigen::Vector3d v);

// Converts a 6-vector [w; v] into an se(3) matrix
Eigen::Matrix<double, 4, 4> vectorTose3(const Eigen::Matrix<double, 6,1> &v);

// Computes the matrix exponential of an se(3) matrix.
Eigen::Matrix<double, 4, 4> MatrixExp6(const Eigen::Matrix<double, 4,4> &se3mat);

// Computes the matrix logarithm of an SE(3) matrix
Eigen::Matrix<double, 4, 4> MatrixLog6(const Eigen::Matrix<double, 4,4> &T);

// 