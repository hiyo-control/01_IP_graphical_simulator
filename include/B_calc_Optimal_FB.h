#ifndef B_CALC_OPTIMAL_FB_H
#define B_CALC_OPTIMAL_FB_H

#include "A_define_IP_parameter.h"

/* Define stucture */
typedef struct Optimal_FB
{
    float F[4];
} Optimal_FB;

typedef struct IP_linear_model
{
    float A[4][4];
    float B[4];
} IP_linear_model;

void calc_Optimal_FB_vector(IP_param* this_rotor, IP_param* this_pnedulum, Optimal_FB* this_Optimal);
void Optimal_FB_show(Optimal_FB* this);

#endif