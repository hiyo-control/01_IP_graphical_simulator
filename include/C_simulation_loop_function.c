#include <stdio.h>
#include <math.h>

#include "A_define_IP_parameter.h"
#include "B_calc_Optimal_FB.h"
#include "C_simulation_loop_function.h"
#include "F_arduino_acceleration.h"

static void calc_dot_state(IP_param* this_rotor, IP_param* this_pendulum, IP_Non_linear_model* this_model, accel_val* this_accel);

void state_initialize(IP_Non_linear_model* this)
{
    this->state[0] = 0;
    this->state[1] = 30*M_PI/180;
    this->state[2] = 0;
    this->state[3] = 0;

    this->control_input = 0;
}

void state_show(IP_Non_linear_model* this, float T)
{
    printf("Time = %.3f", T);
    printf(",_");
    printf("state=[%.3f, %.3f, %.3f, %.3f]", 180/M_PI*this->state[0], 180/M_PI*this->state[1], 180/M_PI*this->state[2], 180/M_PI*this->state[3]);
    printf(",_");
    printf("Input = %.3f", this->control_input);
    printf("\n");
}

void calc_Control_input(Optimal_FB* this_Optimal, IP_Non_linear_model* this_model)
{
    float control_input_buf = 0;

    for (int i=0; i<4; i++)
    {
        control_input_buf = control_input_buf - this_Optimal->F[i]*this_model->state[i];
    }

    this_model->control_input = control_input_buf;
}

void calc_Next_state(IP_param* this_rotor, IP_param* this_pendulum, IP_Non_linear_model* this_model, float delta_T, accel_val* this_accel)
{
    // Define inner variable
    float k1[4], k2[4], k3[4], k4[4];
    float state_sub[4];

    // save state
    for (int i=0; i<4; i++)
    {
        state_sub[i] = this_model->state[i];
    }

    // Calculate k1 vector
    calc_dot_state(this_rotor, this_pendulum, this_model, this_accel);
    for (int i=0; i<4; i++)
    {
        k1[i] = delta_T*this_model->dot_state[i];
        this_model->state[i] = state_sub[i] + k1[i]/2;
    }

    // Calculate k2 vector
    calc_dot_state(this_rotor, this_pendulum, this_model, this_accel);
    for (int i=0; i<4; i++)
    {
        k2[i] = delta_T*this_model->dot_state[i];   
        this_model->state[i] = state_sub[i] + k2[i]/2;
    }

    // Calculate k3 vector
    calc_dot_state(this_rotor, this_pendulum, this_model, this_accel);
    for (int i=0; i<4; i++)
    {
        k3[i] = delta_T*this_model->dot_state[i];   
        this_model->state[i] = state_sub[i] + k3[i];
    }

    // Calculate k4 vector
    calc_dot_state(this_rotor, this_pendulum, this_model, this_accel);
    for (int i=0; i<4; i++)
    {
        k4[i] = delta_T*this_model->dot_state[i];
    }

    // Calc Nest state by Runge Kutta
    for(int i=0; i<4; i++)
    {
        this_model->state[i] = state_sub[i] + (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) / 6;
    }
}

void calc_dot_state(IP_param* this_rotor, IP_param* this_pendulum, IP_Non_linear_model* this_model, accel_val* this_accel)
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

    // Define inner variable
    static float g = 9.8;
    float state_buf[4];
    float m[2][2];
    float d[2][2];
    float h[2];
    float det_M;

    // Calculate m[2][2], d[2][2], h[2] term
    state_buf[0] = this_model->state[0];
    state_buf[1] = this_model->state[1];
    state_buf[2] = this_model->state[2];
    state_buf[3] = this_model->state[3];

    m[0][0] = Jr + mp*lr*lr + Jp*sin( state_buf[1] )*sin( state_buf[1] ); m[1][0] = mp*lr*0.5*lp*cos( state_buf[1] );
    m[0][1] = mp*lr*0.5*lp*cos( state_buf[1] );                           m[1][1] = Jp;
    
    det_M = m[0][0]*m[1][1] - m[1][0]*m[0][1];

    d[0][0] = dr; d[1][0] =  0;
    d[0][1] =  0; d[1][1] = dp;

    h[0] = Jp*sin( 2*state_buf[1] )*state_buf[2]*state_buf[3]                     + mp*lr*0.5*lp*sin( state_buf[1] )*state_buf[3]*state_buf[3];
    h[1] = Jp*sin(   state_buf[1] )*cos( state_buf[1] )*state_buf[2]*state_buf[2] + mp*0.5*lp*g*sin( state_buf[1] ) + 0.001*this_accel->Accl / lp;

    // Calculate f_term of IP Non lienar model
    this_model->f_term[0] = state_buf[2];
    this_model->f_term[1] = state_buf[3];
    this_model->f_term[2] = ( -m[1][1]*d[0][0]*state_buf[2] + m[1][0]*d[1][1]*state_buf[3] ) / det_M + (  h[0]*m[1][1] - h[1]*m[1][0] ) / det_M;
    this_model->f_term[3] = (  m[1][0]*d[0][0]*state_buf[2] - m[0][0]*d[1][1]*state_buf[3] ) / det_M + ( -h[0]*m[1][0] + h[1]*m[0][0] ) / det_M;

    // Calculate g_term of IP Non lienar model
    this_model->g_term[0] = 0;
    this_model->g_term[1] = 0;
    this_model->g_term[2] =  m[1][1] / det_M;
    this_model->g_term[3] = -m[0][1] / det_M;

    for (int i=0; i<4; i++)
    {
        this_model->dot_state[i] = this_model->f_term[i] + this_model->g_term[i]*this_model->control_input;
    }
}
