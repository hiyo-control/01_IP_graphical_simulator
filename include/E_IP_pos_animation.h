#ifndef ANIME_H
#define ANIME_H

#include "C_simulation_loop_function.h"

typedef struct IP_posture
{
    float rotor_pos[2][3];
    float pendulum_pos[2][3];
} IP_posture;

void IP_pos_initialize(IP_posture* this_pos, IP_Non_linear_model* this_model, float T);
void IP_pos_show(IP_posture* this_pos, IP_Non_linear_model* this_model, float T);

#endif