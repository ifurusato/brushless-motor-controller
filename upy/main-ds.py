# main.py -- put your code here!

import time
from display import Display

PERSIST = False

display = Display()
display.hello(persist=PERSIST)
time.sleep(1)

try:
    while PERSIST:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    display.close()

