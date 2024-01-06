#include <iostream>
#include <vector>
#include "calc_P_matrix.h"
#include "calc_Optimal_Linear_FB_vector.h"
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

MatrixXf calc_P_matrix(MatrixXf A, MatrixXf B, MatrixXf Q, MatrixXf R)
{
    /* Define M matrix */
    /*int n = (int)A.rows();*/
    MatrixXf M(8, 8);

    M << A, -B*R.inverse()*B.transpose(), -Q, -A.transpose();

    /* Calculate Eigen structure of M matrix */
    EigenSolver<Matrix<float,8,8>> eigensolver(M);
    if ( eigensolver.info() != Success ) {
        abort();
    }

    /* Pick up stable eigen vector */
    MatrixXcf M_eigen_vector_stable(8, 4);

    int j = 0;
    for ( int k=0; k<8; k++ ) {
        if ( eigensolver.eigenvalues().real()(k) < 0 ){
            M_eigen_vector_stable.col(j) = eigensolver.eigenvectors().block(0, k, 8, 1);

            j++;
        }
    }

    /* Define u & v vector */
    MatrixXcf u_matrix(4, 4), v_matrix(4, 4);
    u_matrix = M_eigen_vector_stable.block(0, 0, 4, 4);
    v_matrix = M_eigen_vector_stable.block(4, 0, 4, 4);
    
    /* Calculate P matrix */
    MatrixXf P_calc(4, 4);
    P_calc = ( v_matrix * u_matrix.inverse() ).real();

    return P_calc;
}