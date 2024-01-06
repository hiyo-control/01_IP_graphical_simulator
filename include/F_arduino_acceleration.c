#include <stdio.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#include "F_arduino_acceleration.h"

static void get_uart_data(accel_val* this);
static void calc_acceleration(accel_val* this);

void accel_construct(accel_val* this)
{
    /* シリアルポートオープン */
    this->fd = serialOpen("/dev/ttyACM0",9600);    
    
    wiringPiSetup();
    fflush(stdout);
    
    if(this->fd<0){
        printf("can not open serialport");
    }
    delay(500);
}

void get_acceleration(accel_val* this)
{
    get_uart_data(this);
    calc_acceleration(this);
}

void get_uart_data(accel_val* this)
{
    this->val = serialGetchar(this->fd);
    if(this->val != -1)
    {
        if(this->val == 'H')
        {
            this->switch_value = serialGetchar(this->fd);
            for (int i=0; i<6; i++)
            {
                this->data[i] = serialGetchar(this->fd);
            }
        }
    }
    else
    {
        printf("no data\n");
    }
}

void calc_acceleration(accel_val* this)
{
    if(this->val != -1)
    {
        float xAccl_buf = ( (this->data[1]*256) + (this->data[0]&0xF0) ) / 16;
        if(xAccl_buf  > 2047)
        {
            xAccl_buf  -= 4096;
        }

        float yAccl_buf  = ( (this->data[3]*256) + (this->data[2]&0xF0) ) / 16;
        if(yAccl_buf  > 2047)
        {
            yAccl_buf  -= 4096;
        }

        float zAccl_buf  = ( (this->data[5]*256) + (this->data[4]&0xF0) ) / 16;
        if(zAccl_buf  > 2047)
        {
            zAccl_buf  -= 4096;
        }

        this->xAccl = xAccl_buf  * 0.0098;
        this->yAccl = yAccl_buf  * 0.0098;
        this->zAccl = zAccl_buf  * 0.0098;

        float xAccl_Sq = pow(this->xAccl, 2);
        float yAccl_Sq = pow(this->yAccl, 2);
        float zAccl_Sq = pow(this->zAccl, 2);
        float Accl_Sq_sum = xAccl_Sq + yAccl_Sq + zAccl_Sq;

        if(this->switch_value == 1)
        {
            this->Accl = pow(Accl_Sq_sum, 0.5) - 9.84;
        }
        else
        {
            this->Accl = 0;
        }
    }
    else
    {
        printf("no acceleration data\n");
    }
}

void acceleration_show(accel_val* this)
{
    printf("Accl = %.3f\n", this->Accl);
}