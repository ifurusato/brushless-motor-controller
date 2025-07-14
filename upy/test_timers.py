import pyb
from pyb import Pin

# Define available timers and their channels with pin names
TIMERS = {
    1: {
        1: ["A8", "E9"],
        2: ["A9", "E11"],
        3: ["A10", "E13"],
    },
    2: {
        1: ["A0", "A15"],
        2: ["A1"],
        3: ["A2"],
        4: ["A3"],
    },
    3: {
        1: ["A6", "B4"],
        2: ["A7", "B5"],
    },
    4: {
        1: ["B6", "D12"],
        2: ["B7", "D13"],
        3: ["B8", "D14"],
        4: ["B9", "D15"],
    },
    5: {
        1: ["A0"],
        2: ["A1"],
        3: ["A2"],
        4: ["A3"],
    },
    8: {
        1: ["C6"],
        2: ["C7"],
        3: ["C8"],
        4: ["C9"],
    }
}

def test_timer(timer_id, channels):
    try:
        timer = pyb.Timer(timer_id, freq=1000)
        print(f"Timer {timer_id} initialized.")
    except Exception as e:
        print(f"Failed to initialize Timer {timer_id}: {e}")
        return

    for ch_num, pin_list in channels.items():
        for pin_name in pin_list:
            try:
                pin_obj = getattr(Pin.board, pin_name)
                ch = timer.channel(ch_num, pyb.Timer.PWM, pin=pin_obj)
                print(f"  Channel {ch_num} OK on {pin_name}")
                break  # success on first available pin
            except AttributeError:
                print(f"  Failed to configure CH{ch_num} on Pin {pin_name}: pin not found on board")
            except Exception as e:
                print(f"  Failed to configure CH{ch_num} on Pin {pin_name}: {e}")

def main():
    for timer_id, channels in TIMERS.items():
        test_timer(timer_id, channels)

main()

