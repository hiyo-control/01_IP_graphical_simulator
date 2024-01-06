#ifndef _cplusplus

namespace cpp_func
{
    extern "C" {
        #endif

        extern float F_cpp[4];
        void calc_Optimal_Linear_FB_vector(float model_A[4][4], float model_B[4], float design_Q[4][4], float design_R);

        #ifndef _cplusplus
    };
};
#endif
