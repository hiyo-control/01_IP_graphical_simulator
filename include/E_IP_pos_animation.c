#include <stdio.h>
#include <math.h>

#include "C_simulation_loop_function.h"
#include "E_IP_pos_animation.h"
#include "F_arduino_acceleration.h"

FILE *gp;

static void calc_IP_posture(IP_posture* this_pos, IP_Non_linear_model* this_model);

void IP_pos_initialize(IP_posture* this_pos, IP_Non_linear_model* this_model, float T)
{
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [-1.5:1.5]\n");
    fprintf(gp, "set yrange [-1.5:1.5]\n");
    fprintf(gp, "set zrange [-1  :1  ]\n");
    fprintf(gp, "set ticslevel 0\n");
    fprintf(gp, "set grid;\n");
    
    IP_pos_show(this_pos, this_model, T);
}

void IP_pos_show(IP_posture* this_pos, IP_Non_linear_model* this_model, float T)
{
    calc_IP_posture(this_pos, this_model);
    
    fprintf(gp, "set title 'Time=%.1f'\n", T);

    // Axial
    fprintf(gp, "splot '-' with lines linetype 2\n");
    fprintf(gp, "%.2f %.2f %.2f\n", 0.0, 0.0, -1.0);
    fprintf(gp, "%.2f %.2f %.2f\n", 0.0, 0.0,  0.2);

    // Rotor
    fprintf(gp, "splot '-' with lines linetype 2\n");
    fprintf(gp, "%.2f %.2f %.2f\n", this_pos->rotor_pos[0][0], this_pos->rotor_pos[0][1], this_pos->rotor_pos[0][2]);
    fprintf(gp, "%.2f %.2f %.2f\n", this_pos->rotor_pos[1][0], this_pos->rotor_pos[1][1], this_pos->rotor_pos[1][2]);

    // Pendulum
    fprintf(gp, "splot '-' with lines linetype 2\n");
    fprintf(gp, "%.2f %.2f %.2f\n", this_pos->pendulum_pos[0][0], this_pos->pendulum_pos[0][1], this_pos->pendulum_pos[0][2]);
    fprintf(gp, "%.2f %.2f %.2f\n", this_pos->pendulum_pos[1][0], this_pos->pendulum_pos[1][1], this_pos->pendulum_pos[1][2]);
    
    fprintf(gp, "e\n");
    fflush(gp);
}

void calc_IP_posture(IP_posture* this_pos, IP_Non_linear_model* this_model)
{
    // deg->rad converter
    float deg2rad = M_PI/180;

    // Define length of Rotor & Pendulum
    float rotor_l    = 1;
    float pendulum_l = 1;

    // Define angle of Rotor & Pendulum
    float rotor_angle    = this_model->state[0] - 90*deg2rad;
    float pendulum_angle = this_model->state[1];

    // Calculate Rotor posture
    this_pos->rotor_pos[0][0] = 0;
    this_pos->rotor_pos[0][1] = 0;
    this_pos->rotor_pos[0][2] = 0;

    this_pos->rotor_pos[1][0] = rotor_l * cos(rotor_angle);
    this_pos->rotor_pos[1][1] = rotor_l * sin(rotor_angle);
    this_pos->rotor_pos[1][2] = 0;

    // Calculate Pendulum posture
    this_pos->pendulum_pos[0][0] = this_pos->rotor_pos[1][0];
    this_pos->pendulum_pos[0][1] = this_pos->rotor_pos[1][1] - 0.2*pendulum_l * sin(pendulum_angle);
    this_pos->pendulum_pos[0][2] = this_pos->rotor_pos[1][2] - 0.2*pendulum_l * cos(pendulum_angle);

    this_pos->pendulum_pos[1][0] = this_pos->rotor_pos[1][0];
    this_pos->pendulum_pos[1][1] = this_pos->rotor_pos[1][1] + 0.8*pendulum_l * sin(pendulum_angle);
    this_pos->pendulum_pos[1][2] = this_pos->rotor_pos[1][2] + 0.8*pendulum_l * cos(pendulum_angle);
}