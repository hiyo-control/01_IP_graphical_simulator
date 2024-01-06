#ifndef A_DEFINE_PARAMETER_H
#define A_DEFINE_PARAMETER_H

/* Define stucture */
typedef struct IP_param
{
    float mass;
    float length;
    float viscosity;
    float inertia;
} IP_param;

void Rotor_param_construct(IP_param* this);
void Pendulum_param_construct(IP_param* this);

#endif