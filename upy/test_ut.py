
from pyb import UART, Timer, Pin

UART1 = False
UART2 = False
UART3 = False
UART4 = False # problem
UART5 = False
UART6 = False
UART7 = False

TIMER1 = False
TIMER2 = False
TIMER3 = False
TIMER4 = False # problem
TIMER5 = False
TIMER8 = False


# Timer1 initialized with channels on A8, A9, A10
# Timer2 initialized with channels on A0, A1, A2, A3
# Timer3 initialized with channels on A6, A7
# Timer4 initialized with channels on B6, B7, B8, B9
# Timer5 initialized with channels on A0, A1, A2, A3
# Timer8 initialized with channels on C6, C7, C8, C9

a0 = Pin('A0')
a1 = Pin('A1')
a2 = Pin('A2')
a3 = Pin('A3')
a6 = Pin('A6')
a7 = Pin('A7')
a8 = Pin('A8')
a9 = Pin('A9')    # UART1_TX
a10 = Pin('A10')  # UART1_RX

c6 = Pin('C6')
c7 = Pin('C7')
c8 = Pin('C8')
c9 = Pin('C9')

print('pins setup')

print('uarts ............................................ ')
if UART1:
    try:
        uart1 = UART(1) # tx=Pin('A9'), rx=Pin('A10'))
        print("UART1 initialized on TX=A9, RX=A10")
    except Exception as e:
        print("UART1 init failed:", e)
if UART2:
    try:
        uart2 = UART(2) # tx=Pin('A2'), rx=Pin('A3'))
        print("UART2 initialized on TX=A2, RX=A3")
    except Exception as e:
        print("UART2 init failed:", e)
if UART3:
    try:
        uart3 = UART(3) # tx=Pin('B10'), rx=Pin('B11'))
        print("UART3 initialized on TX=B10, RX=B11")
    except Exception as e:
        print("UART3 init failed:", e)
if UART4:
    try:
        uart4 = UART(4) # tx=Pin('C11'), rx=Pin('C10'))
        print("UART4 initialized on TX=C11, RX=C10")
    except Exception as e:
        print("UART4 init failed:", e)
if UART5:
    try:
        uart5 = UART(5) # tx=Pin('B12'), rx=Pin('B13'))
        print("UART5 initialized on TX=B12, RX=B13")
    except Exception as e:
        print("UART5 init failed:", e)
if UART6:
    try:
        uart6 = UART(6) # tx=Pin('C6'), rx=Pin('C7'))
        print("UART6 initialized on TX=C6, RX=C7")
    except Exception as e:
        print("UART6 init failed:", e)
if UART7:
    try:
        uart7 = UART(7) # tx=Pin('E8'), rx=Pin('E7'))
        print("UART7 initialized on TX=E8, RX=E7")
    except Exception as e:
        print("UART7 init failed:", e)

print('timers ............................................ ')
if TIMER1:
    try:
        timer1 = Timer(1, freq=1000)
        timer1.channel(1, Timer.PWM, pin=Pin('A8'))
        timer1.channel(2, Timer.PWM, pin=Pin('A9'))
        timer1.channel(3, Timer.PWM, pin=Pin('A10'))
        print("Timer1 initialized with channels on A8, A9, A10")
    except Exception as e:
        print("Timer1 init failed:", e)
if TIMER2:
    try:
        timer2 = Timer(2, freq=1000)
        timer2.channel(1, Timer.PWM, pin=Pin('A0'))
        timer2.channel(2, Timer.PWM, pin=Pin('A1'))
        timer2.channel(3, Timer.PWM, pin=Pin('A2'))
        timer2.channel(4, Timer.PWM, pin=Pin('A3'))
        print("Timer2 initialized with channels on A0, A1, A2, A3")
    except Exception as e:
        print("Timer2 init failed:", e)
if TIMER3:
    try:
        timer3 = Timer(3, freq=1000)
        timer3.channel(1, Timer.PWM, pin=Pin('A6'))
        timer3.channel(2, Timer.PWM, pin=Pin('A7'))
        print("Timer3 initialized with channels on A6, A7")
    except Exception as e:
        print("Timer3 init failed:", e)
if TIMER4:
    try:
        timer4 = Timer(4, freq=1000)
        timer4.channel(1, Timer.PWM, pin=Pin('B6'))
        timer4.channel(2, Timer.PWM, pin=Pin('B7'))
        timer4.channel(3, Timer.PWM, pin=Pin('B8'))
        timer4.channel(4, Timer.PWM, pin=Pin('B9'))
        print("Timer4 initialized with channels on B6, B7, B8, B9")
    except Exception as e:
        print("Timer4 init failed:", e)
if TIMER5:
    try:
        timer5 = Timer(5, freq=1000)
        timer5.channel(1, Timer.PWM, pin=Pin('A0'))
        timer5.channel(2, Timer.PWM, pin=Pin('A1'))
        timer5.channel(3, Timer.PWM, pin=Pin('A2'))
        timer5.channel(4, Timer.PWM, pin=Pin('A3'))
        print("Timer5 initialized with channels on A0, A1, A2, A3")
    except Exception as e:
        print("Timer5 init failed:", e)
if TIMER8:
    try:
        timer8 = Timer(8, freq=1000)
        timer8.channel(1, Timer.PWM, pin=Pin('C6'))
        timer8.channel(2, Timer.PWM, pin=Pin('C7'))
        timer8.channel(3, Timer.PWM, pin=Pin('C8'))
        timer8.channel(4, Timer.PWM, pin=Pin('C9'))
        print("Timer8 initialized with channels on C6, C7, C8, C9")
    except Exception as e:
        print("Timer8 init failed:", e)

