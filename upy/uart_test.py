
from machine import UART

for uart_num in range(1, 6):
    try:
        uart = UART(uart_num, baudrate=9600)
        uart.write('UART{} test OK\r\n'.format(uart_num))
        print('UART{} initialized and message sent.'.format(uart_num))
    except Exception as e:
        print('UART{} failed: {}'.format(uart_num, e))

print('complete.')

