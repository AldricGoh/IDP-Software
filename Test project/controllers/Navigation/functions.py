from controller import Robot, Motor, Keyboard, Compass, GPS
import numpy as np
import time
from setup import *

def robotLocation():
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]
    return coord2d
 
def compassAngle():
    compassValues = compass.getValues()
    rad = -np.arctan2(compassValues[0],compassValues[2])
    return round(rad, 2) #+- sign essential
    
def dist(x1,z1,x2,z2):
    return (x2-x1)**2+(x2-x1)**2
    
def dist_to_wall():
    #Finds what the distance sensor should be reading
    angle = compassAngle()
    position = gps.getValues()
    x, z = position[0], position[1]
    wallN = abs((1.2-0.01 - x) / np.sin(angle))#0.01 terms as wall has a thickness
    wallS = abs((-1.2+0.01 - x) / np.sin(angle))
    wallW = abs((-1.2+0.01 - z) / np.cos(angle))
    wallE = abs((+1.2-0.01 - z) / np.cos(angle))
    if (angle>=0 and angle <= np.pi/2):
        return min([wallN,wallE,1.6])
    elif (angle>=np.pi/2 and angle <= np.pi):
        return round(min([wallW,wallN,1.6]), 2)
    if (angle<0 and angle >= -np.pi/2):
        return round(min([wallS,wallW,1.6]), 2)
    elif (angle<=-np.pi/2 and angle > -np.pi):
        return round(min([wallE,wallS,1.6]), 2)
    else:
        return 1

def get_object_position(position,angle,sensordist):
    #Determines the coordinates of the box
    x = position[0]
    z = position[1]
    return [x+(sensordist+0.025) * np.sin(angle),z+(sensordist+0.025) * np.cos(angle)]#Adds to centroid, may need to be tuned

def checkIfBox():
    turn()
    print(ds_bottom.getValue(), dist_to_wall())
    if ds_bottom.getValue() < dist_to_wall():
        print('Object found at:' + str(compassAngle()) + str(sensor_to_dist()))
        robot.step(1)
    robot.step(1)
        

def moveToPosition(position):
    #moves wheels to position (in rads)
    left_wheel.setPosition(position)
    right_wheel.setPosition(position)

def turn():
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(-MAX_SPEED)
    right_wheel.setVelocity(MAX_SPEED)

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
    
def align(direction):
    currentAngle = compassAngle()
    print(currentAngle)
    if currentAngle != direction:
        turn()
        robot.step(1)
    else:
        print('aligned')
        robotStatus = 'aligned'
        robot.step(1)
    
def passive_wait(time):
    start_time = robot.getTime()
    while start_time + time > robot.getTime():
        print(start_time + time)
        print(robot.getTime())
        robot.step(1)