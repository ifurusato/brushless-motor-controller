import pyb
import gc

def list_uarts():
    print("uarts:")
    for i in range(1, 9):
        try:
            pyb.UART(i)
            print(f"  - UART{i}")
        except Exception:
            pass

def list_timers():
    print("timers:")
    for i in range(1, 24):
        try:
            pyb.Timer(i)
            print(f"  - TIM{i}")
        except Exception:
            pass

def list_pin_afs():
    print("pins:")
    for name in dir(pyb.Pin.board):
        pin = getattr(pyb.Pin.board, name)
        try:
            afs = pin.af_list()
            if afs:
                print(f"  {name}:")
                for af in afs:
                    print(f"    - af: {af[0]}")
                    print(f"      fn: {af[1]}")
        except Exception:
            pass

def main():
    list_uarts()
    print()
    list_timers()
    print()
    list_pin_afs()
    gc.collect()

main()

