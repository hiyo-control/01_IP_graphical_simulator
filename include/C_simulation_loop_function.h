#ifndef C_SIMULATION_LOOP_FUNCTION_H
#define C_SIMULATION_LOOP_FUNCTION_H

#include "A_define_IP_parameter.h"
#include "B_calc_Optimal_FB.h"
#include "F_arduino_acceleration.h"

typedef struct IP_Non_linear_model
{
    float dot_state[4];
    float state[4];
    float f_term[4];
    float g_term[4];
    float control_input;
} IP_Non_linear_model;

void state_initialize(IP_Non_linear_model* this);
void state_show(IP_Non_linear_model* this, float T);
void calc_Control_input(Optimal_FB* this_Optimal, IP_Non_linear_model* this_model);
void calc_Next_state(IP_param* this_rotor, IP_param* this_pendulum, IP_Non_linear_model* this_model, float delta_T, accel_val* this_accle);

#endif