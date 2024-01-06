#include <iostream>
#include <vector>
#include "calc_P_matrix.h"
#include "calc_Optimal_Linear_FB_vector.h"
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

namespace cpp_func
{
    float F_cpp[4];
    extern "C" void calc_Optimal_Linear_FB_vector(float model_A[4][4], float model_B[4], float design_Q[4][4], float design_R)
    {
        /* Define A, B, Q, R matrix */
        MatrixXf A(4, 4), Q(4, 4);
        MatrixXf B(4, 1);
        MatrixXf R(1, 1);

        for (int i=0; i<4; i++){
            for (int j=0; j<4; j++){
                A(i, j) = model_A[i][j];
                Q(i, j) = design_Q[i][j]; 
            }
            B(i, 0) = model_B[i];
        }
        R(0, 0) = design_R;

        /* Calculate P matrix*/
        MatrixXf P = calc_P_matrix(A, B, Q, R);

        /* Calculate Optimal FB gain vector */
        MatrixXf F_buf = R.inverse()*B.transpose()*P;

        for (int k=0; k<4; k++ ){
            F_cpp[k] = F_buf(0,k);
        }

        return;
    }
};

