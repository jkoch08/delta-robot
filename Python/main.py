import time

from controller import *

#
# Main function for the computer side of the delta robot. 
#

def loop():
    """
    Keeps looping until user enters something besides a playback command.
    """
    message = raw_input()
    if message == 'p':
        ct.replay = True
        loop()
    else:
        ct.stop = True

if __name__ == "__main__":                  
    ct = ControllerThread("ct")
    time.sleep(0.5)
    if ct.serConnected and ct.leapController.is_connected:
        ct.start()
        raw_input()
        ct.stop = True
        if ct.playback:
            loop()
        