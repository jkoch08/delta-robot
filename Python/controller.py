from workspace import *
from math import *
import time
import threading
import Leap

#
# Processes input from the Leap Motion and uses it to send data to a Delta
# Robot.
#

# Constants, for vector manipulation.
x = 0
y = 1
z = 2

def getFingerPos(leapController):
    """
    Uses the Leap motion to get a raw value of the finger position in mm in the
    coordinate system of the device (x, y, z). If no finger is detected,
    returns (0, 0, 0).
    """
    if not leapController.is_connected:
        print "Connection lost."
        return (0, 0, 0)
    frame        = leapController.frame()
    fingers      = frame.fingers
    indexFingers = fingers.finger_type(Leap.Finger.TYPE_INDEX)
    indexFinger  = indexFingers[0]
    distalBone   = indexFinger.bone(Leap.Bone.TYPE_DISTAL)
    distalPos    = distalBone.next_joint
    
    return (distalPos.x, distalPos.y, distalPos.z)


def transformPoint(p):
    """
    Takes a position 'p' = (x, y, z) in millimeters in the coordinate system 
    of the leapmotion, and outputs a point (x, y, z) in inches in the coordinate
    system of the of the Delta Robot by doing an appropriate transformation.
    """
    (xIn, yIn, zIn) = p
    # First, change the axes.
    (xOut, yOut, zOut) = (xIn, -zIn, yIn)
    # Next, convert to inches. 
    mmPerInch = 25.4
    (xOut, yOut, zOut) = (xOut / mmPerInch, yOut / mmPerInch, zOut / mmPerInch)
    # Next, translate the origin.
    xOffset = 0.0
    yOffset = 0.0
    zOffset = -20.0
    (xOut, yOut, zOut) = (xOut + xOffset, yOut + yOffset, zOut + zOffset)
    # Finally, apply a scale factor.
    xScale = 3.0
    yScale = 3.0
    zScale = 0.75
    (xOut, yOut, zOut) = (xScale * xOut, yScale * yOut, zScale * zOut)
    # TODO EDIT SCALE, OFFSET VALUES.
    return (xOut, yOut, zOut)

def restrainMove(pStart, pEnd):
    """
    'pStart' and 'pEnd' are points of the form (x, y, z), in inches. If pEnd
    is <= 1 inch away from pStart, returns pEnd. Otherwise, returns the point
    1 inch from 'pStart' in the direction of 'pEnd'.
    """
    if dist(pStart, pEnd) <= 1:
        return pEnd
    else:
        dirVector = dirTo(pStart, pEnd)     # Vector from 'pStart' to 'pEnd'
        normDirVector = normalize(dirVector)    # Norm of 'dirVector'
        transVector = sclProd(1, normDirVector) # Actual vector to translate;
                                                # equal to 1" * 'dirVector'
        return (pStart[x] + transVector[x],
                pStart[y] + transVector[y],
                pStart[z] + transVector[z])

# 
# VECTOR FUNCTIONS.
# 
def dirTo(p1, p2):
    """
    Returns the vector v = [vx, vy, vz] that points from point 
    'p1' = (x1, y1, z1) to point 'p2' = (x2, y2, z2).
    """
    (x1, y1, z1) = p1
    (x2, y2, z2) = p2
    return [x2 - x1, y2 - y1, z2 - z1]

def norm(v):
    """
    Returns the norm of vector 'v' = [vx, vy, vz].
    """
    return sqrt(v[x] ** 2 + v[y] ** 2 + v[z] ** 2)

def normalize(v):
    """
    Returns the normalized version of vector 'v' = [vx, vy, vz].
    """
    n = norm(v)
    return [v[x] / n, v[y] / n, v[z] / n]

def dist(p1, p2):
    """
    Returns the distance from 'p1' = (x1, y1, z1) to 'p2' = (x2, y2, z2).
    """
    return norm(dirTo(p1, p2))

def sclProd(s, v):
    """
    Returns a vector consisting of scalar 's' multiplied by vector 'v'.
    """
    output = len(v) * [0]
    for i in range(len(v)):
        output[i] = s * v[i]
    return output
                     
class ControllerThread(threading.Thread):
    """
    This class provides a thread that, when enabled, gets hand data from the 
    leap motion, processes it, and sends it to the Delta Robot.
    """
    
    def __init__(self, threadName):
        """
        Creates a new ControllerThread named 'threadName'.
        """
        threading.Thread.__init__(self, name = threadName)       
        self.stop = False
        
        # Current estimated position of robot. 
        self.currentPos = HOME # Starts by default at home upon powerup.
        
        # Leap Controller.
        self.leapController = Leap.Controller()
        while not self.leapController.is_connected:
            time.sleep(0.05)
    
    def run(self):
        """
        Causes thread to run indefinitely, until 'self.stop' is set True.
        """ 
        while not self.stop:
            posOut = self.currentPos # Position to be outputted to move to.
                                     # By default, output current position
                                     # (don't move).
####            print "START:      " + str(self.currentPos)
            fingerPos = getFingerPos(self.leapController)
####            print "FINGER:     " + str(fingerPos)
            
            if fingerPos != (0.0, 0.0, 0.0): # Data from LeapMotion is good            
                desiredPos = transformPoint(fingerPos) # Desired move position
####                print "DESIRED:    " + str(desiredPos)
                boundedPos = boundDestination(self.currentPos, desiredPos)
                print "BOUNDED:    " + str(boundedPos)
                restrainedPos = restrainMove(self.currentPos, boundedPos)
                    # Make move distance no more than 1" for mechanism safety.
####                print "RESTRAINED: " + str(restrainedPos
####                print "\n"
                posOut = restrainedPos

            # TODO SEND 'posOut' TO THE ROBOT
####            print "OUT:         " + str(posOut)
            self.currentPos = posOut # Save position just sent to robot.
            time.sleep(0.10) # Sloppy way to ensure robot gets to position.
                             # Better would be to have feedback.
