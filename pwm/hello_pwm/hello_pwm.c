/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Output PWM signals on pins 0 and 1

#include "pico/stdlib.h"
#include "hardware/pwm.h"

int i = 0;

int main() {
    /// \tag::setup_pwm[]

    // // Tell GPIO 0 and 1 they are allocated to the PWM
    // gpio_set_function(0, GPIO_FUNC_PWM);
    // // gpio_set_function(1, GPIO_FUNC_PWM);

    // // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    // uint slice_num = pwm_gpio_to_slice_num(0);

    // // Set period of 4 cycles (0 to 3 inclusive)
    // pwm_set_wrap(slice_num, 500);
    // // Set channel A output high for one cycle before dropping
    // pwm_set_chan_level(slice_num, PWM_CHAN_A, 250);
    // // Set initial B output high for three cycles before dropping
    // // pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
    // // Set the PWM running
    // pwm_set_enabled(slice_num, true);
    // /// \end::setup_pwm[]

    // // Note we could also use pwm_set_gpio_level(gpio, x) which looks up the
    // // correct slice and channel for a given GPIO.

    gpio_set_function(1,GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(1);

    pwm_set_enabled(slice_num,true);

    //for 100 HZ  0.01/8ns 
    pwm_set_wrap(slice_num,1250000);

    // pwm_set_chan_level(slice_num,PWM_CHAN_A,250);

    // i++;
    // pwm_set_chan_level(slice_num,PWM_CHAN_A,i/10*500);
    // sleep_ms(300);

    // if(i>10){
    //     i=0;
    // }

    while (1)
    {
    
          i++;
        pwm_set_chan_level(slice_num,PWM_CHAN_A,i/10*1250000);
        sleep_ms(300);

        if(i>10){
            i=0;
        }
        // tight_loop_contents();
    }
    
}
