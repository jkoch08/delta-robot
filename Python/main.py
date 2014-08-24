import time

from controller import *

#
# Main function for the computer side of the delta robot. 
#


if __name__ == "__main__":                  
    ct = ControllerThread("ct")
    ct.start()
    raw_input()
    ct.stop = True