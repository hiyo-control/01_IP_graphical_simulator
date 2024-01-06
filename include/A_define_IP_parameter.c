#include <stdio.h>
#include <math.h>

#include "A_define_IP_parameter.h"

void Rotor_param_construct(IP_param* this_rotor)
{
    this_rotor->mass      = 0.057;
    this_rotor->length    = 0.137*2;
    this_rotor->viscosity = 2.919*pow(10,-3);
    this_rotor->inertia   = 2.67 *pow(10,-4);
}

void Pendulum_param_construct(IP_param* this_pendulum)
{
    this_pendulum->mass      = 0.067;
    this_pendulum->length    = 0.155*2;
    this_pendulum->viscosity = 1.671*pow(10,-4);
    this_pendulum->inertia   = 0.971*pow(10,-3); 
}
