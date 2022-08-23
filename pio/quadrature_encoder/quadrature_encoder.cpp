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

        output_pwm = output_pwm + 4*(220-delta);

        output_pwm = output_pwm >  65000 ? 65000:output_pwm; //65025
        output_pwm = output_pwm < 0 ?  1 : output_pwm;  

        delta_last = delta ; 
             
        pwm_set_chan_level(slice_num, PWM_CHAN_A, output_pwm); 

        printf("position %8d, delta %6d   ,  %6d  \n  ", new_value,  delta , output_pwm);
 
        sleep_ms(50);
    }
}

