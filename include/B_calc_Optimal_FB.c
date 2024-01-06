#include <stdio.h>
#include <math.h>

#include "A_define_IP_parameter.h"
#include "B_calc_Optimal_FB.h"

static float g = 9.8;

static void IP_linear_model_construct(IP_param* this_rotor, IP_param* this_pendulum, IP_linear_model* this_model);
static void A_model_show(IP_linear_model* this);
static void B_model_show(IP_linear_model* this);

void calc_Optimal_FB_vector(IP_param* this_rotor, IP_param* this_pendulum, Optimal_FB* this_Optimal)
{
    IP_linear_model IP_linear_model;

    IP_linear_model_construct(this_rotor, this_pendulum, &IP_linear_model);
    A_model_show(&IP_linear_model);
    B_model_show(&IP_linear_model);

    this_Optimal->F[0] = -0.1432;
    this_Optimal->F[1] = -1.0577;
    this_Optimal->F[2] = -0.0863;
    this_Optimal->F[3] = -0.1355;
}

void IP_linear_model_construct(IP_param* this_rotor, IP_param* this_pendulum, IP_linear_model* this_model)
{
    // Define parameter of Rotor
    float lr = this_rotor->length;
    float dr = this_rotor->viscosity;
    float Jr = this_rotor->inertia;

    // Define parameter of Rotor
    float mp     = this_pendulum->mass;
    float lp     = this_pendulum->length;
    float dp     = this_pendulum->viscosity;
    float Jp_sub = this_pendulum->inertia;
    float Jp     = Jp_sub + mp * 0.5*lp * 0.5*lp;

    // Define sub variable
    float L1   = mp * lr * 0.5*lp;
    float L2   = Jr + mp * lr * lr;
    float L3   = mp * 0.5*lp * g;
    float detM = Jp*L2 - L1*L1;
    
    // System matrix
    this_model->A[0][0] = 0; this_model->A[0][1] =  0         ; this_model->A[0][2] =  1         ; this_model->A[0][3] =  0         ;
    this_model->A[1][0] = 0; this_model->A[1][1] =  0         ; this_model->A[1][2] =  0         ; this_model->A[1][3] =  1         ;
    this_model->A[2][0] = 0; this_model->A[2][1] = -L1*L3/detM; this_model->A[2][2] = -dr*Jp/detM; this_model->A[2][3] =  dp*L1/detM;
    this_model->A[3][0] = 0; this_model->A[3][1] =  L2*L3/detM; this_model->A[3][2] =  dr*L1/detM; this_model->A[3][3] = -dp*L2/detM;

    // Input dstribute vector
    this_model->B[0] =  0      ;
    this_model->B[1] =  0      ;
    this_model->B[2] =  Jp/detM;
    this_model->B[3] = -L1/detM;
}

void A_model_show(IP_linear_model* this)
{
    printf("System Matrix of Inverted Pendulum ------------------\n");
    printf("A[4][4]=\n");
    printf("[%f, %f, %f, %f] \n", this->A[0][0], this->A[0][1], this->A[0][2], this->A[0][3]);
    printf("[%f, %f, %f, %f] \n", this->A[1][0], this->A[1][1], this->A[1][2], this->A[1][3]);
    printf("[%f, %f, %f, %f] \n", this->A[2][0], this->A[2][1], this->A[2][2], this->A[2][3]);
    printf("[%f, %f, %f, %f] \n", this->A[3][0], this->A[3][1], this->A[3][2], this->A[3][3]);
    printf("\n");
}

void B_model_show(IP_linear_model* this)
{
    printf("Input destribute vector of Inverted Pendulum --------\n");
    printf("B[4]=\n");
    printf("[%f] \n", this->B[0]);
    printf("[%f] \n", this->B[1]);
    printf("[%f] \n", this->B[2]);
    printf("[%f] \n", this->B[3]);
    printf("\n");
}

void Optimal_FB_show(Optimal_FB* this)
{
    printf("Optimal FB vector ------------------------------------\n");
    printf("F[4]=\n");
    printf("[%f] \n", this->F[0]);
    printf("[%f] \n", this->F[1]);
    printf("[%f] \n", this->F[2]);
    printf("[%f] \n", this->F[3]);
    printf("\n");
}