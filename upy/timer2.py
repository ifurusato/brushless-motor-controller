from pyb import UART, Timer, Pin

try:
    timer2 = Timer(2, freq=1000)
    timer2.channel(1, Timer.PWM, pin=Pin('A0'))
    timer2.channel(2, Timer.PWM, pin=Pin('A1'))
    timer2.channel(3, Timer.PWM, pin=Pin('A2'))
    timer2.channel(4, Timer.PWM, pin=Pin('A3'))
    print("Timer2 initialized with channels on A0, A1, A2, A3")
except Exception as e:
    print("Timer2 init failed:", e)

