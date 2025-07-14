from pyb import UART, Timer, Pin



try:
    pinB6 = Pin('B6')
    pinB7 = Pin('B7')
    pinB8 = Pin('B8')
    pinB9 = Pin('B9')
    print("pins initialized.")
    timer4 = Timer(4, freq=1000)
    timer4.channel(1, Timer.PWM, pin=pinB6)
    timer4.channel(2, Timer.PWM, pin=pinB7)
    timer4.channel(3, Timer.PWM, pin=pinB8)
    timer4.channel(4, Timer.PWM, pin=pinB9)
    print("Timer4 initialized.")
    print("Pins set up: B6, B7, B8, B9")
except Exception as e:
    print("Timer4 init failed:", e)

