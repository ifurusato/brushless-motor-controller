# X9 -> PB6    TIM4
# X9 -> PB6    TIM4

from pyb import Pin, Timer
import time

def main():

    print("start…")

    INVERT = False
    timer4 = None
    ch1    = None
    try:
        pwm_pin = Pin('PB6')
        timer4 = Timer(4, freq=20000)
        ch1 = timer4.channel(1, Timer.PWM, pulse_width_percent=0, pin=pwm_pin)
        while True:
            for duty_percent in range(0, 100, 5):
                if INVERT:
                    inverted_duty = 100 - duty_percent
                    ch1.pulse_width_percent(inverted_duty)
                else:
                    compare_value = int((duty_percent / 100.0) * timer4.period())  # Calculate compare value
                    ch1.pulse_width(compare_value)  # Set compare value directly
                print("running at {}% duty cycle".format(duty_percent))
                time.sleep(0.25)
            time.sleep(1)
            for duty_percent in range(100, 0, -5):
                if INVERT:
                    inverted_duty = 100 - duty_percent
                    ch1.pulse_width_percent(inverted_duty)
                else:
                    compare_value = int((duty_percent / 100.0) * timer4.period())  # Calculate compare value
                    ch1.pulse_width(compare_value)  # Set compare value directly
                print("running at {}% duty cycle".format(duty_percent))
                time.sleep(0.25)
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nctrl-caught.")
    except Exception as e:
        print("{} raised: {}".format(type(e), e))
    finally:
        print("stopping motor…")
        if ch1:
            ch1.pulse_width_percent(100)
        print("stopping timer…")
        if timer4:
            timer4.deinit()
        print("motor stopped.")

if __name__ == '__main__':
    main()

