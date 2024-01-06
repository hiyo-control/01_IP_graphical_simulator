#ifndef CALC_P_MATRIX_H
#define CALC_P_MATRIX_H

#include <iostream>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/Eigenvalues>

using namespace Eigen;

MatrixXf calc_P_matrix(MatrixXf A, MatrixXf B, MatrixXf Q, MatrixXf R);

#endif