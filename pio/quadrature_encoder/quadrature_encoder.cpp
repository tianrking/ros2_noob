/**
 * Copyright (c) 2021 pmarques-dev @ github
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "quadrature_encoder.pio.h"

//
// ---- quadrature encoder interface example
//
// the PIO program reads phase A/B of a quadrature encoder and increments or
// decrements an internal counter to keep the current absolute step count
// updated. At any point, the main code can query the current count by using
// the quadrature_encoder_*_count functions. The counter is kept in a full
// 32 bit register that just wraps around. Two's complement arithmetic means
// that it can be interpreted as a 32-bit signed or unsigned value, and it will
// work anyway.
//
// As an example, a two wheel robot being controlled at 100Hz, can use two
// state machines to read the two encoders and in the main control loop it can
// simply ask for the current encoder counts to get the absolute step count. It
// can also subtract the values from the last sample to check how many steps
// each wheel as done since the last sample period.
//
// One advantage of this approach is that it requires zero CPU time to keep the
// encoder count updated and because of that it supports very high step rates.
//


int wrap;
int GPIO_motor_pwm=6;

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

typedef struct
{
    //PID运算模式
    uint8_t mode;
    //PID 三个基本参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //PID最大输出
    float max_iout; //PID最大积分输出

    float set;	  //PID目标值
    float fdb;	  //PID当前值

    float out;		//三项叠加输出
    float Pout;		//比例项输出
    float Iout;		//积分项输出
    float Dout;		//微分项输出
    //微分项最近三个值 0最新 1上一次 2上上次
    float Dbuf[3];  
    //误差项最近三个值 0最新 1上一次 2上上次
    float error[3];  

} pid_type_def;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

// void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)

void PID_init(pid_type_def *pid)
{
    // if (pid == NULL || PID == NULL)
    // {
    //     return;
    // }
    // pid->mode = mode;
    // pid->Kp = PID[0];
    // pid->Ki = PID[1];
    // pid->Kd = PID[2];

    pid->mode = 0;
    pid->Kp  = 100;
    pid->Ki  = 0;
    pid->Kd  = 0.2;

    pid->max_out = 62000;
    pid->max_iout = 10000;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float PID_calc(pid_type_def *pid, float ref, float set)
{
    //判断传入的PID指针不为空
    if (pid == NULL)
    {
        return 0.0f;
    }
    //存放过去两次计算的误差值
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    //设定目标值和当前值到结构体成员
    pid->set = set;
    pid->fdb = ref;
    //计算最新的误差值
    pid->error[0] = set - ref;
    //判断PID设置的模式
    // if (pid->mode == PID_POSITION)
    if (pid->mode == 0)
    {
        //位置式PID
        //比例项计算输出
        pid->Pout = pid->Kp * pid->error[0];
        printf("pid->Pout  %8d \n" ,pid->Pout );
        //积分项计算输出
        pid->Iout += pid->Ki * pid->error[0];
        //存放过去两次计算的微分误差值
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        //当前误差的微分用本次误差减去上一次误差来计算
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        //微分项输出
        pid->Dout = pid->Kd * pid->Dbuf[0];
        //对积分项进行限幅
        // LimitMax(pid->Iout, pid->max_iout);
        //叠加三个输出到总输出
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        //对总输出进行限幅
        // LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        //增量式PID
        //以本次误差与上次误差的差值作为比例项的输入带入计算
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        //以本次误差作为积分项带入计算
        pid->Iout = pid->Ki * pid->error[0];
        //迭代微分项的数组
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        //以本次误差与上次误差的差值减去上次误差与上上次误差的差值作为微分项的输入带入计算
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        //叠加三个项的输出作为总输出
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        //对总输出做一个先限幅
        // LimitMax(pid->out, pid->max_out);
	}
	return pid->out;
}

void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }
	//当前误差清零
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    //微分项清零
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    //输出清零
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    //目标值和当前值清零
    pid->fdb = pid->set = 0.0f;
}

int main() {
    int new_value, delta, old_value = 0;

    // Base pin to connect the A phase of the encoder.
    // The B phase must be connected to the next pin
    const uint PIN_AB = 10;

    stdio_init_all();

    PIO pio = pio0;
    const uint sm = 0;

    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm, offset, PIN_AB, 0);

    gpio_set_function(GPIO_motor_pwm, GPIO_FUNC_PWM); 
    uint slice_num = pwm_gpio_to_slice_num(GPIO_motor_pwm);
    wrap = 62499 ; // 2khz
    // wrap = 12499 ; //should be 10 khz right
    // wrap = 1249 ; // should be 100 khz right
    pwm_set_wrap(slice_num,wrap);
    pwm_set_enabled(slice_num, true);
 
    int i = 0 ;
    int flag = 0;

    pid_type_def *diy_pid;

    const float _PID[] = {100,1,1};
    const float max_out = 60000;
    // PID_init(diy_pid,PID_POSITION,_PID,60000,60000);

    PID_init(diy_pid);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 30000); 

    float last_value = 1;

    int _pwm = i * wrap/10 ;

    int delta_last;

    float output_pwm=12000;

    while (1) {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        new_value = quadrature_encoder_get_count(pio, sm);
        delta = new_value - old_value;
        old_value = new_value;

        // if(flag){
        //     i++;
        //     if(i>10){
        //         flag = 0;
        //         i=10;
        //     }
        // }
        // else{
        //     i--;
        //     if(i<0){
        //         flag = 1;
        //         i=0;
        //     }
        // }
        

        // pwm_set_chan_level(slice_num, PWM_CHAN_A, diy_pid->out*wrap/100);
        
        // printf("position %8d, delta %6d   ,  %6d    %6d  \n  ", new_value,  delta , PID_calc(diy_pid, delta , 450),PID_calc(diy_pid, 450 ,delta));
        // printf("position %8d, delta %6d  \n  ", new_value);    


        output_pwm = output_pwm + 4*(220-delta);

        output_pwm = output_pwm >  65000 ? 65000:output_pwm; //65025
        output_pwm = output_pwm < 0 ?  1 : output_pwm;  

        delta_last = delta ; 
        
        // PID_calc(diy_pid, delta , 450);

        // float _kk  = PID_calc(diy_pid, delta , 450);

        
            
    
        pwm_set_chan_level(slice_num, PWM_CHAN_A, output_pwm); 

        printf("position %8d, delta %6d   ,  %6d  \n  ", new_value,  delta , output_pwm);
 
        sleep_ms(50);
    }
}

