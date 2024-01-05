#include <stdio.h>
#include <unistd.h>

#include "./include/A_define_IP_parameter.h"
#include "./include/B_calc_Optimal_FB.h"
#include "./include/C_simulation_loop_function.h"
#include "./include/D_time_sampling.h"
#include "./include/E_IP_pos_animation.h"
#include "./include/F_arduino_acceleration.h"

int main(void)
{
    // ---------------------------------------------------------------------
    // Define IP_param structure
    IP_param rotor, pendulum;
    Rotor_param_construct(&rotor);
    Pendulum_param_construct(&pendulum);

    // ---------------------------------------------------------------------
    // Calculate Optimal FB gain vector
    Optimal_FB Optimal_FB;
    calc_Optimal_FB_vector(&rotor, &pendulum, &Optimal_FB);
    Optimal_FB_show(&Optimal_FB);

    // ---------------------------------------------------------------------
    // Calculate Optimal FB gain vector
    accel_val accel_val;
    accel_construct(&accel_val);

    // ---------------------------------------------------------------------
    // Define simulation condition
    float delta_T  = 0.01;
    float end_time = 30;
    float data_num = end_time / delta_T;
    float T        = 0;
    int   sample   = 1;

    // ---------------------------------------------------------------------
    // Initialize simulation
    IP_Non_linear_model IP_Non_linear_model;
    state_initialize(&IP_Non_linear_model);
    calc_Control_input(&Optimal_FB, &IP_Non_linear_model);

    time_struct time_struct;
    time_initialize(&time_struct, delta_T);

    IP_posture IP_pos;
    IP_pos_initialize(&IP_pos, &IP_Non_linear_model, T);

    // ---------------------------------------------------------------------
    // Show animation
    printf("Start simulation\n");
    while(!getchar());

    get_start_time(&time_struct);

    while( sample <= (int)data_num )
    {
        get_Now_time(&time_struct);
        int check = check_time(&time_struct, delta_T);

        if(check)
        {
        }
        else
        {
            T = sample*delta_T;
            
            get_acceleration(&accel_val);
            //acceleration_show(&accel_val);
            //state_show(&IP_Non_linear_model, T);
            IP_pos_show(&IP_pos, &IP_Non_linear_model, T);
            calc_Control_input(&Optimal_FB, &IP_Non_linear_model);
            calc_Next_state(&rotor, &pendulum, &IP_Non_linear_model, delta_T, &accel_val);

            get_prev_time(&time_struct);
            sample = sample + 1;
        }
    }

    get_end_time(&time_struct);
    printf("End simulation\n");
    printf("--------------------------------------------------------------------\n");
    time_show(&time_struct);
}