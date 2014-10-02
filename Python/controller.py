from workspace import *
from math import *
import time
import threading
import serial
import Leap

#
# Processes input from the Leap Motion and uses it to send data to a Delta
# Robot.
#

# Constants, for vector manipulation.
x = 0
y = 1
z = 2

# Constants for mechanism control.

MAX_POS  = 10000 # Max number of positions to record

numAvgs = 25 # number of times to average average hand pos

restrainThresh = 0.15 # inches  # Best: 0.12
restrainDist   = 0.15 # inches  # Best: 0.12
sleepTime      = 0.01 # seconds # Best: 0.01

xOffset = 0.0
yOffset = 0.0
zOffset = -20.0

                # Last best:
xScale = -1.0   # 1.0   
yScale = -1.0   # 1.0  
zScale =  1.0   # 1.0  

def getFingerPos(leapController):
    """
    OBSOLTE -- Use getHandPos instead.
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
    
def getHandPos(leapController):
    """
    Uses the Leap motion to get a raw value of the center of the palm in mm 
    in the coordinate system of the device (x, y, z). If no finger is detected,
    returns (0, 0, 0).
    """
    if not leapController.is_connected:
        print "Connection lost."
        return (0, 0, 0)
    frame        = leapController.frame() # Frame
    hands        = frame.hands            # HandList
    hand         = hands.frontmost        # Hand
    handPos      = hand.palm_position     # Vector
    return (handPos.x, handPos.y, handPos.z)

def getAvgPos(leapController):
    """
    Averages the hand and index finger positions. If either one is not detected,
    returns (0, 0, 0). Averages numAvgs times.
    """
    if not leapController.is_connected:
        print "Connection lost."
        return (0, 0, 0)
    trials = []
    for i in range(numAvgs):
        fingerPos = getFingerPos(leapController)
        handPos = getHandPos(leapController)
        if fingerPos == (0, 0, 0) or handPos == (0, 0, 0):
            return (0, 0, 0)
        else: # Got rid of handPos 
            trials.append(((fingerPos[x] + fingerPos[x]) / 2.0,
                          (fingerPos[y] + fingerPos[y]) / 2.0,
                          (fingerPos[z] + fingerPos[z]) / 2.0))
    return avgPoints(trials)

def avgPoints(lst):
    """
    Averages a list 'lst' of tuples, each of form (x, y, z).
    """
    xSum = 0
    ySum = 0
    zSum = 0
    
    for point in lst:
        xSum += point[x]
        ySum += point[y]
        zSum += point[z]
    
    return (xSum / len(lst), ySum / len(lst), zSum / len(lst))
                 
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
    (xOut, yOut, zOut) = (xOut + xOffset, yOut + yOffset, zOut + zOffset)
    # Finally, apply a scale factor.
    (xOut, yOut, zOut) = (xScale * xOut, yScale * yOut, zScale * zOut)
    # TODO EDIT SCALE, OFFSET VALUES.
    return (xOut, yOut, zOut)

def restrainMove(pStart, pEnd):
    """
    'pStart' and 'pEnd' are points of the form (x, y, z), in inches. If pEnd
    is <= 1 inch away from pStart, returns pEnd. Otherwise, returns the point
    1 inch from 'pStart' in the direction of 'pEnd'.
    """
    if dist(pStart, pEnd) <= restrainThresh:
        return pEnd
    else:
        dirVector = dirTo(pStart, pEnd)     # Vector from 'pStart' to 'pEnd'
        normDirVector = normalize(dirVector)    # Norm of 'dirVector'
        transVector = sclProd(restrainDist, normDirVector) 
                # Actual vector to translate; equal to 1" * 'dirVector'.
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

def add(p, v):
    """
    Adds to point 'p' the vector 'v'.
    """
    return (p[x] + v[x], p[y] + v[y], p[z] + v[z])

def sclProd(s, v):
    """
    Returns a vector consisting of scalar 's' multiplied by vector 'v'.
    """
    output = len(v) * [0]
    for i in range(len(v)):
        output[i] = s * v[i]
    return output

def lineImg(p1, p2, step):
    """
    Returns a list of tuples representing the points from 'p1' to 'p2',
    incrementing by size 'step'.
    """
    img = []
    currentPos = p1
    direction = dirTo(p1, p2)
    while dist(currentPos, p2) >= step * 1.00: # Add 0% for rounding error
        img.append((currentPos[x], currentPos[y], currentPos[z]))
        currentPos = add(currentPos, sclProd(step, direction))
    img.append(p2)
    return img
    
                     
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
        self.handTrack = True; # If controller should track
        self.record    = True; # If controller should record while tracking
        self.playback  = True; # If controller should playback after stopping
        self.stop = False      # indicates controler should stop
        self.replay = False    # indicates controller is should replay
        self.serConnected = False
        
        # Current estimated position of robot. 
        self.currentPos = HOME # Starts by default at home upon powerup.
        
        # Serial object.
        self.ser = serial.Serial()
        self.ser.baudrate = 57600
        self.ser.port = 5
        try: 
            self.ser.open()
            time.sleep(0.1) # Wait for serial to open
            self.serConnected = True
        except Exception as e:
            print "COULD NOT CONNECT OVER SERIAL."
            return
        
        # Leap Controller.
        self.leapController = Leap.Controller()
        time.sleep(0.4) # Wait for leap to connect
        if not self.leapController.is_connected:
            print "LEAP MOTION IS NOT CONNECTED."
            
    def outputImage(self, img):
        """
        Outputs the image 'img', which consists of a list of tuples of the form
        (x, y, z).
        """
        for p in img:
            self.outputPosition(p)
            if self.stop:
                return
        self.outputPosition(HOME)
        print("'p' to playback - Enter to cancel")
            
            
    def outputPosition(self, p):
        """
        Outputs the position 'p' = (x, y, z) over the serial.
        """
        # First, generate the string to be outputted. 
        (x, y, z) = p
        output = ("$" +
                  str(int(round(x * 1000))) +
                  "," +
                  str(int(round(y * 1000))) +
                  "," +                  
                  str(int(round(z * 1000))) +
                  "*y")
        # Write over the serial line.
##        print(output)
        self.ser.write(output)
        time.sleep(sleepTime)
        
    def listenToHand(self):
        """
        Listens to hand position information and outputs it to the Delta.
        """
        posOut = self.currentPos # Position to be outputted to move to.
                                 # By default, output current position
                                 # (don't move).
####        print "START:      " + str(self.currentPos)
        pos = getAvgPos(self.leapController)
####        print "FINGER:     " + str(fingerPos)
            
        if pos != (0.0, 0.0, 0.0): # Data from LeapMotion is good            
            desiredPos = transformPoint(pos) # Desired move position
####            print "DESIRED:    " + str(desiredPos)
            boundedPos = boundDestination(self.currentPos, desiredPos)
####            print "BOUNDED:    " + str(boundedPos)
            restrainedPos = restrainMove(self.currentPos, boundedPos)
                # Make move distance no more than 1" for mechanism safety.
####            print "RESTRAINED: " + str(restrainedPos
####            print "\n"
            posOut = restrainedPos

        # SEND 'posOut' TO THE ROBOT, if serial connected.
        if self.serConnected:
            self.outputPosition(posOut)
        print "%.3f   %.3f   %.3f" % (posOut[x], posOut[y], posOut[z])

        self.currentPos = posOut # Save position just sent to robot.
    
    def run(self):
        """
        Causes thread to run indefinitely, until 'self.stop' is set True.
        """         
       
        posCount = 0; # Number of positions recorded.
        positions = []
        
        # Hand Tracking.
        if self.handTrack:
            while not self.stop:
                self.listenToHand()
                if self.record:
                    if posCount < MAX_POS:
                        positions.append(self.currentPos)
                        posCount += 1
            if self.playback:
                print("'p' to playback - Enter to cancel")
                self.stop = False # Resets -- awaiting a new True command
                # Needs to dwell and wait for response from main thread.
                while not self.stop:
                    if self.replay:
                        self.replay = False
                        self.outputImage(positions)
        
        self.ser.close() # Close serial after thread is killed.
