#!/micropython

import time

from pixel import Pixel
from colors import *

pixel = None

try:

    count = 8
    pixel = Pixel(pixel_count=count, brightness=0.1)

    pixel.set_color(color=COLOR_RED)
    time.sleep(0.2)
    pixel.set_color(color=COLOR_GREEN)
    time.sleep(0.2)
    pixel.set_color(color=COLOR_BLUE)
    time.sleep(0.2)

    # set_color(self, index=0, color=None):
    while True:
        for index in range(0,8):
            print('index: {}'.format(index))
            pixel.set_color(index=index, color=COLOR_RED)
            time.sleep(0.1)
            pixel.set_color(index=index, color=COLOR_GREEN)
            time.sleep(0.1)
            pixel.set_color(index=index, color=COLOR_BLUE)
            time.sleep(0.1)
            pixel.set_color(index=index, color=COLOR_BLACK)
            time.sleep(0.1)

except KeyboardInterrupt:
    print('Ctrl-C caught; exiting…')
except Exception as e:
    print('{} raised in main loop: {}'.format(type(e), e))
finally:
    if pixel:
        pixel.off()
    print('complete.')

#EOF
