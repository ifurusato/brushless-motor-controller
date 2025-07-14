
import time
from sysfont import sysfont
from st7735py import TFT
from machine import SPI, Pin

# Initialize SPI and TFT
spi = SPI(4, baudrate=31250000)
Pin('E10', Pin.OUT)  # Backlight on
tft = TFT(spi, 'E13', 'D15', 'E11')  # SPI, DC, RST, CS
tft.rgb(True)
tft.initr()
tft.invertcolor(True)
tft.rotation(3)

# Screen constants
SCREEN_HEIGHT = 160
SCREEN_WIDTH = 80
X_OFFSET = 10

# Font layout mapping: font size -> (row height, y_start)
LAYOUTS = {
    1.0: (15, 35),
    1.5: (18, 35),
    2.0: (24, 30),
    2.5: (30, 25),
    3.0: (36, 20),
}

def show_line(text, line=1, size=1.5, clear=False):
    """
    Display text on a given line using layout from LAYOUTS dict.

    Args:
        text (str): Text to display.
        line (int): 1-based line number.
        size (float): Font size (must exist in LAYOUTS).
        clear (bool): Whether to clear the screen before drawing.
    """
    if size not in LAYOUTS:
        raise ValueError("Font size {} not configured in LAYOUTS".format(size))

    row_height, y_start = LAYOUTS[size]
    max_lines = (SCREEN_HEIGHT - y_start) // row_height

    if not (1 <= line <= max_lines):
        raise ValueError("Line number must be between 1 and {} for font size {}".format(max_lines, size))

    if clear:
        tft.fill(TFT.BLACK)

    y = y_start + (line - 1) * row_height
    pos = (X_OFFSET, y)

    print("-- text: '{}'; line: {}; pos: {}".format(text, line, pos))
    tft.text(pos, text, TFT.WHITE, sysfont, size)

# Example usage
show_line("row 1 is magic", line=1, size=1.5, clear=True)
show_line("row 2 is boring", line=2, size=1.5)
show_line("row 3 is lost", line=3, size=1.5)
show_line("row 4 is roaring", line=4, size=1.5)

