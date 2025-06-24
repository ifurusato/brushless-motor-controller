# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal

# USB Mode options for pyb.usb_mode():
#
# 'VCP'       - USB Serial (Virtual COM Port) only
#               Use this if you want REPL and serial communication over USB.
#
# 'MSC'       - USB Mass Storage only
#               Use this if you want the board to appear as a USB drive (no serial).
#
# 'VCP+MSC'   - Both USB Serial and Mass Storage
#               Most common for development: access REPL and files simultaneously.
#
# 'CDC'       - Synonym for 'VCP' on many boards (USB serial only).
#
# None        - Disable USB entirely (no serial, no storage).
#
# For most development, 'VCP+MSC' is recommended to allow both REPL and file access.
#
# Example usage:
# import pyb
# pyb.usb_mode('VCP+MSC')

import pyb

pyb.country('US') # ISO 3166-1 Alpha-2 code, eg US, GB, DE, AU
#pyb.usb_mode('CDC')
pyb.usb_mode('VCP+MSC')

#pyb.main('main.py') # main script to run after this one
