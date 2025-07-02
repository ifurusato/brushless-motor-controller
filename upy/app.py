# a convenient alias for testing

import sys
#import main 

# Remove the cached module
#sys.modules.pop('app', None)

def reload_module(name):
    import sys
    if name in sys.modules:
        del sys.modules[name]
    return __import__(name)

main = reload_module('uart_slave_app')
main.exec()

