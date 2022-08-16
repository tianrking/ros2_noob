#include "pico/stdlib.h"
#include "hardware/pwm.h"
 
int wrap ;
int main() {
    gpio_set_function(0, GPIO_FUNC_PWM);
 
    uint slice_num = pwm_gpio_to_slice_num(0);
    // 125Mhz / (3+1)  for 31.25Mhz
    // pwm_set_wrap(slice_num, 3); 
    // 25% duty cycle  1/(3+1)
    // pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);    



    
    // wrap = 124999; //should be  1khz  2khz??? why 65535 clear
    wrap = 62499 ; // 2khz
    // wrap = 12499 ; //should be 10 khz right
    // wrap = 1249 ; // should be 100 khz right
    // wrap = 65534; // should be 1mhz right

    pwm_set_wrap(slice_num,wrap);
    pwm_set_enabled(slice_num, true);
 
    int i = 0 ;
    int flag = 0;
    while (1) {
    	// just ouptut PWM
        if(flag){
            i++;
            if(i>10){
                flag = 0;
                i=10;
            }
        }
        else{
            i--;
            if(i<0){
                flag = 1;
                i=0;
            }
        }
        pwm_set_chan_level(slice_num, PWM_CHAN_A, i*wrap/10); 
        sleep_ms(1000); 
    }
}

