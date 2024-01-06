#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "D_time_sampling.h"

struct timespec real_start, real_end, real_sample;

void time_initialize(time_struct* this, float delta_T)
{
    this->sample_time = delta_T;
    this->start_time = 0;
    this->now_time   = 0;
    this->prev_time  = -delta_T;
    this->end_time   = 0;

    this->check = 0;
}

void get_start_time(time_struct* this)
{
    clock_gettime(CLOCK_REALTIME, &real_start);

    int sec  = real_start.tv_sec ;
    int nsec = real_start.tv_nsec;
    
    this->start_time = (double)sec + (double)nsec / (1000 * 1000 * 1000);
}

void get_Now_time(time_struct* this)
{
    clock_gettime(CLOCK_REALTIME, &real_sample);
    
    int sec  = real_sample.tv_sec  - real_start.tv_sec ;
    int nsec = real_sample.tv_nsec - real_start.tv_nsec;
    
    this->now_time = (double)sec + (double)nsec / (1000 * 1000 * 1000);
}

void get_prev_time(time_struct* this)
{
    this->prev_time = this->now_time;
}

void get_end_time(time_struct* this)
{
    clock_gettime(CLOCK_REALTIME, &real_end);
    
    int sec  = real_end.tv_sec  - real_start.tv_sec ;
    int nsec = real_end.tv_nsec - real_start.tv_nsec;
    
    this->end_time = (double)sec + (double)nsec / (1000 * 1000 * 1000);
}

int check_time(time_struct* this, float delta_T)
{
    if(this->now_time - this->prev_time < delta_T)
    {
        this->check = 1;
    }
    else
    {
        this->check = 0;
    }

    return this->check;
}

void time_show(time_struct* this)
{
    printf("simulation time = %f\n", this->end_time    );
    printf("sample_time = %f\n"    , this->sample_time );
}