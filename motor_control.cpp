#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "iostream"

#include "quadrature_encoder.pio.h"

int wrap;
int GPIO_motor_pwm = 6;

class PID_class
{
private:
    float kp, ki, kd;
    float err, last_err;
    float err_i;
    float err_d;
    float fix;

public:
    PID_class(int p, int i, int d)
    {
        kp = p;
        ki = i;
        kd = d;
        std::cout << kp << " " << ki << " " << kd;
    };

    void PID_init(int p, int i, int d)
    {
        PID_class(p, i, d);
    };

    void PID_change(int p, int i, int d)
    {
        kp = p;
        ki = i;
        kd = d;
        std::cout << kp << " " << ki << " " << kd;
    };

    int caculate(int now, int target)
    {
        err = target - now;
        err_i += err;
        err_d = err - last_err;
        last_err = err;

        fix = kp * err + ki * err_i + kd * err_d;
        return fix;
    }

} PID(5, 0.5, 0);

int main()
{
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
    wrap = 62499; // 2khz
    // wrap = 12499 ; //should be 10 khz right
    // wrap = 1249 ; // should be 100 khz right
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true);

    pwm_set_chan_level(slice_num, PWM_CHAN_A, 30000);

    float output_pwm = 0; // 12000

    while (1)
    {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        new_value = quadrature_encoder_get_count(pio, sm);
        delta = new_value - old_value;
        old_value = new_value;

        output_pwm += PID.caculate(delta, 12);

        if (output_pwm > 65000)
        {
            output_pwm = 65000;
        }

        if (output_pwm < 0)
        {
            output_pwm = 0;
        }

        pwm_set_chan_level(slice_num, PWM_CHAN_A, output_pwm);

        std::cout << "position " << new_value << " delta " << delta << "  pwm " << output_pwm*100/62500 <<"%" << std::endl;

        sleep_ms(2);
    }
}
