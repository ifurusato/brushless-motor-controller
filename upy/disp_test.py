
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# source: https://github.com/jkorte-dev/micropython-board-STM32H723VGT6
#
#  from sysfont import sysfont
#  import vga1_16x16
#  import vga1_bold_16x16
#  import vga1_8x8
#  import vga1_bold_16x32
#  import vga1_8x16
#  
#  import utime
#  import st7789
#  import tft_config
#  import vga1_8x8 as font1
#  import vga1_8x16 as font2
#  import vga1_bold_16x16 as font3
#  import vga1_bold_16x32 as font4
#  
#    BLACK = 0
#    RED    = TFTColor(0xFF, 0x00, 0x00)
#    MAROON = TFTColor(0x80, 0x00, 0x00)
#    GREEN  = TFTColor(0x00, 0xFF, 0x00)
#    FOREST = TFTColor(0x00, 0x80, 0x80)
#    BLUE   = TFTColor(0x00, 0x00, 0xFF)
#    NAVY   = TFTColor(0x00, 0x00, 0x80)
#    CYAN   = TFTColor(0x00, 0xFF, 0xFF)
#    YELLOW = TFTColor(0xFF, 0xFF, 0x00)
#    PURPLE = TFTColor(0xFF, 0x00, 0xFF)
#    WHITE  = TFTColor(0xFF, 0xFF, 0xFF)
#    GRAY   = TFTColor(0x80, 0x80, 0x80)

import os
import time
from sysfont import sysfont as SysFont
from st7735py import TFT
import romanp
import NotoSans_32
from machine import SPI, Pin

spi = SPI(4, baudrate=31250000)
Pin('E10', Pin.OUT) # init backlight

tft=TFT(spi,'E13','D15','E11') # spi, aDC, aReset, aCS
tft.rgb(True)
tft.initr()
# settings for small 0.96'' 80x160 display
tft.invertcolor(True)
tft.rotation(3)

# when using sysfont at size 1, 4 rows are: 35, 50, 65, 80

FONT_FAMILY = SysFont
FONT_SIZE = 1.5
ROW_HEIGHT = 15
row = 35
ROW_1 = (10, row)
row += ROW_HEIGHT
ROW_2 = (10, row)
row += ROW_HEIGHT
ROW_3 = (10, row)
row += ROW_HEIGHT
ROW_4 = (10, row)

def show(text, pos=(50,50), size=FONT_SIZE, clear=True):
    if clear:
        tft.fill(TFT.BLACK)
    print("-- text: '{}'; pos: {}".format(text, pos))
    tft.text(pos, text, TFT.WHITE, FONT_FAMILY, size)

show('row 1 is magic', ROW_1, clear=True)
show('row 2 is boring', ROW_2, clear=False)
show('row 3 is lost', ROW_3, clear=False)
show('row 4 is roaring', ROW_4, clear=False)

print('done.')
