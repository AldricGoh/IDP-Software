from controller import Robot, Motor, Keyboard, Compass
import numpy as np
import time

start = time.time()
print('start')
#Useful constants
TIME_STEP = 64
MAX_SPEED = 3

#ROBOT CONSTANTS
WHEEL_RADIUS = 5.08
ROBOT_WIDTH = 18 + 2*WHEEL_RADIUS

# --DO NOT EDIT--
TURN_RADIUS = (ROBOT_WIDTH-WHEEL_RADIUS+1.11)/2
# --DO NOT EDIT--

robot = Robot()

#get robot motor devices
left_wheel = robot.getDevice("left_wheel")
right_wheel = robot.getDevice("right_wheel")
left_door = robot.getDevice("left_door")
right_door = robot.getDevice("right_door")

left_wheel.setVelocity(MAX_SPEED)
right_wheel.setVelocity(MAX_SPEED)
    
#get robot sensor devices
ds_bottom = robot.getDevice('ds_long')
ds_top= robot.getDevice('ds_short')
ls_red = robot.getDevice('ls_red')
ls_green = robot.getDevice('ls_green')
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")

#enable sensors
ds_bottom.enable(TIME_STEP)
ds_top.enable(TIME_STEP)
ls_red.enable(TIME_STEP) 
ls_green.enable(TIME_STEP)
gps.enable(TIME_STEP)
compass.enable(TIME_STEP)
#Emitter Receiver

timestep = int(robot.getBasicTimeStep())

def robotLocation():
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]
    return coord2d
 
def compassAngle():
    compassValues = compass.getValues()
    rad = -np.arctan2(compassValues[0],compassValues[2])
    return rad #+- sign essential
    
def dist(x1,z1,x2,z2):
    return (x2-x1)**2+(x2-x1)**2

def moveToPosition(position):
    #moves wheels to position (in rads)
    left_wheel.setPosition(position)
    right_wheel.setPosition(position)

def setSpeed(speedL,speedR):
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(speedL*MAX_SPEED)
    right_wheel.setVelocity(speedR*MAX_SPEED)

def turn(speed):
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(-speed*MAX_SPEED)
    right_wheel.setVelocity(speed*MAX_SPEED)

def openDoor():
    left_door.setPosition(1.57)
    right_door.setPosition(1.57)

def closeDoor():
    left_door.setPosition(0)
    right_door.setPosition(0)

#Turns anti-clockwise by x radians
def turnRadian(radians):
    position = (radians*TURN_RADIUS)/(WHEEL_RADIUS)
    left_wheel.setPosition(-position)
    right_wheel.setPosition(position)
    left_wheel.setVelocity(MAX_SPEED)
    right_wheel.setVelocity(MAX_SPEED)
    

def checkObstacles(destination):
    #Destination is coordinates we want to go
    #TURN +-66 degrees about axis of symmetry of robot
    #Scan if there are any obstacles to the location desired
    #Using ds_bottom
    #If no, return False
    #If yes, return True
    #if compass.getValues()
    turnRadian((11*np.pi)/30)
    #if 
    
def align(targetValue):
    print(compassAngle())
    if compassAngle() < targetValue:
        while compassAngle() < targetValue:
            turn(1)
    elif compassAngle() > targetValue:
        while compassAngle() > targetValue:
            turn(-1)
    elif compassAngle() == targetValue:
        print('aligned')
    else:
        pass

# def passive_wait(time):
    # start_time = robot.getTime()
    # while start_time + time > robot.getTime():
        # print(start_time + time)
        # print(robot.getTime())
        # robot.step(1)

while robot.step(timestep) != -1:
    
    align(-2)

    pass
    
