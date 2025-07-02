import time
from pyb import Pin, Timer

def main():

    print("start…")

    M0 = 'PB6'
    M1 = 'PB7'
    M2 = 'PB8'
    M3 = 'PB9' 

    D0 = 'PB12'
    D1 = 'PB14'
    D2 = 'PB3'
    D3 = 'PB5'
    
    INVERT = False
    timer4 = None
    ch1    = None
    try:
        
        dir_pin_name = D3
        dir_pin = Pin(dir_pin_name, Pin.OUT)
        dir_pin.value(1)

        pwm_pin_name = M3
        pwm_pin = Pin(pwm_pin_name, Pin.OUT)

        timer4 = Timer(4, freq=20000)
        ch1 = timer4.channel(1, Timer.PWM, pulse_width_percent=0, pin=pwm_pin)
        print("using PWM pin {}".format(pwm_pin_name))
        while True:
          
            for duty_percent in range(0, 100, 5):
                period = timer4.period()
                if INVERT:
                    inverted_duty = 100 - duty_percent
                    ch1.pulse_width_percent(inverted_duty)
                else:
                    value = int((duty_percent / 100.0) * period)
                    print('value: {}'.format(value))
                    ch1.pulse_width(value)
                print("a. running at {}% duty cycle with period: {}".format(duty_percent, period))
                time.sleep(0.25)
            time.sleep(1)
            for duty_percent in range(100, 0, -5):
                period = timer4.period()
                if INVERT:
                    inverted_duty = 100 - duty_percent
                    ch1.pulse_width_percent(inverted_duty)
                else:
                    value = int((duty_percent / 100.0) * period)
                    ch1.pulse_width(value)
                print("b. running at {}% duty cycle with period: {}".format(duty_percent, period))
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

#EOF
