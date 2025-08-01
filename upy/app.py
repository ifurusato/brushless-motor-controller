# a convenient alias for testing that forgets its self

import sys
from logger import Logger, Level
from colorama import Fore, Style

# enable Emergency Exception Buffer
import micropython
micropython.alloc_emergency_exception_buf(256)

__log = Logger('app', level=Level.INFO)

# Forget the modules you want to reload
for mod in ['free', 'cwd', 'main', 'uart_slave_app']:
    __log.debug("forgetting '{}'".format(mod))
    sys.modules.pop(mod, None)

# Import in the desired order (they now re-execute)
import free
import cwd

# forget app.py itself so it re-runs next time
sys.modules.pop(__name__, None)

__log.info(Style.BRIGHT + '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
__log.info(Style.BRIGHT + 'executing UartSlaveApp…')
#import uart_slave_app

def reload_module(name):
    import sys
    if name in sys.modules:
        del sys.modules[name]
    return __import__(name)

main = reload_module('uart_slave_app')
main.exec()

