
from pyb import Pin

for pinname in dir(Pin.board):
    pin = getattr(Pin.board, pinname)
    try:
        for af in pin.af_list():
            if "USART" in af[1] or "UART" in af[1]:
                print(f"{pinname}: {af}")
    except:
        pass

