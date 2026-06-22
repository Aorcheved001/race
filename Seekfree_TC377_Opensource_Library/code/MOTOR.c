
#include "zf_common_headfile.h"

//-----------------限幅保护部分-------------------//
int32 range_protect( int32 current_value , int32 min , int32 max )
{
    if( current_value >= max )
    {
        return max;
    }
    if( current_value <= min )
    {
        return min;
    }
    else
    {
        return current_value;
    }
}


void Motor_Init(void)
{
    pwm_init(PWM_L, 20000, 0);
    pwm_init(PWM_R, 20000, 0);
    gpio_init(DIR_L, GPO, 1, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, 1, GPO_PUSH_PULL);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机控制
//  @param      motor    0 L    1 R
//  @param      pwm     占空比   0-10000（正值=前进，负值=后退），驱动限幅 4000
//  @return
//-------------------------------------------------------------------------------------------------------------------
void Motor_Ctrl( uint8 motor , int32 pwm )
{
    if( motor == MOTOR_L )
    {
        if( pwm >= 0 )
        {
            gpio_set_level( DIR_L , 0 );
            pwm_set_duty( PWM_L , (uint32)range_protect( pwm , 0 , 4000 ) );
        }
        else
        {
            gpio_set_level( DIR_L , 1 );
            pwm_set_duty( PWM_L ,(uint32)range_protect( -pwm , 0 , 4000 ) );
        }
    }
    else if( motor == MOTOR_R )
    {
        if( pwm >= 0 )
        {
            gpio_set_level( DIR_R , 1 );
            pwm_set_duty( PWM_R , (uint32)range_protect( pwm , 0 , 4000 ) );
        }
        else
        {
            gpio_set_level( DIR_R , 0 );
            pwm_set_duty( PWM_R , (uint32)range_protect( -pwm , 0 , 4000 ) );
        }
    }
}
