#ifndef D_TIME_SAMPLING_H
#define D_TIME_SAMPLING_H

typedef struct time_struct
{
    float sample_time;
    float start_time;
    float now_time;
    float prev_time;
    float end_time;

    int check;
} time_struct;

void time_initialize(time_struct* this, float delta_T);
void get_start_time(time_struct* this);
void get_Now_time(time_struct* this);
void get_prev_time(time_struct* this);
void get_end_time(time_struct* this);

void time_show(time_struct* this);

int check_time(time_struct* this, float delta_T);

#endif