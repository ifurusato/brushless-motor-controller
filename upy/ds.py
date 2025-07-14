
import sys
from logger import Logger, Level
from colorama import Fore, Style

MODULE_NAME = 'display' # module to reload

# enable Emergency Exception Buffer
import micropython
micropython.alloc_emergency_exception_buf(256)

__log = Logger('app', level=Level.INFO)

# Forget the modules you want to reload
for mod in ['free', 'cwd', 'main', MODULE_NAME]:
    __log.debug("forgetting '{}'".format(mod))
    sys.modules.pop(mod, None)

# import in the desired order (they now re-execute)
import free
import cwd

# forget app.py itself so it re-runs next time
sys.modules.pop(__name__, None)

__log.info(Style.BRIGHT + '┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
__log.info(Style.BRIGHT + 'executing {}…'.format(MODULE_NAME))

def reload_module(name):
    import sys
    if name in sys.modules:
        del sys.modules[name]
    return __import__(name)

main = reload_module(MODULE_NAME)
main.exec()

