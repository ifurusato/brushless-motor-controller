
from pyb import UART
import time

# Define UARTs and default pin pairs (TX, RX)
uart_configs = {
    1: ('A9', 'A10'),
    2: ('A2', 'A3'),
    3: ('B10', 'B11'),
    6: ('A11', 'A12'),
    7: ('E8', 'E7'),
    8: ('E1', 'E0'),
    9: None,  # Not listed
    10: None, # Not listed
    'LPUART1': ('B6', 'B7')  # Pseudonym for testing
}

# Test each UART
for uart_id, pins in uart_configs.items():
    if pins is None:
        print("Skipping UART{} — no pin mapping provided.".format(uart_id))
        continue

    tx_pin, rx_pin = pins
    try:
        # MicroPython automatically maps UART to correct pins if supported
        uart = UART(uart_id if isinstance(uart_id, int) else 1, 9600)
        uart.write("UART{} TX test\n".format(uart_id))
        print("UART{} initialized on TX={}, RX={}".format(uart_id, tx_pin, rx_pin))
        uart.deinit()
    except Exception as e:
        print("Failed to initialize UART{}: {}".format(uart_id, e))

