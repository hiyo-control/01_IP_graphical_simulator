#ifndef F_ARUDUINO_ACCELERATION_H
#define F_ARUDUINO_ACCELERATION_H

typedef struct accel_val
{
    int fd ;
    int switch_value;
    char val;
    unsigned int data[6];
    float xAccl;
    float yAccl;
    float zAccl;
    float Accl ;
} accel_val;

void accel_construct(accel_val* this);
void get_acceleration(accel_val* this);
void acceleration_show(accel_val* this);

#endif