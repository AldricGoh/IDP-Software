from controller import Robot, Motor, Keyboard, Compass, GPS
import numpy as np
import time
from setup import *

def longDSReading():
    """Use sensitivity curve to convert distance sensor raw data to m distance"""
    reading = ds_bottom.getValue()
    # Sensitivity curve points
    distances = [0.15,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.3,1.5]
    readings = [2.75,2.51,1.99,1.52,1.25,1.04,0.87,0.79,0.74,0.69,0.6,0.55,0.5,0.47,0.45]
    
    distance = distances[0]
    if(reading <= readings[-1]):
        distance = distances[-1]
        
    # Return interpolated value
    for i in range(len(readings)-1):
        if(readings[i] >= reading and readings[i+1] < reading):
            distance = ((reading - readings[i]) / (readings[i+1] - readings[i]) * (distances[i+1] - distances[i]) + distances[i])
    
    return distance
    
def shortDSReading():
    """Use sensitivity curve to convert distance sensor raw data to m distance"""
    reading = ds_top.getValue()
    # Sensitivity curve points
    distances = [0.05,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.8]
    readings = [3.1,2.26,1.27,0.92,0.75,0.6,0.5,0.45,0.41]
    
    distance = distances[0]
    if(reading <= readings[-1]):
        distance = distances[-1]
        
    # Return interpolated value
    for i in range(len(readings)-1):
        if(readings[i] >= reading and readings[i+1] < reading):
            distance = ((reading - readings[i]) / (readings[i+1] - readings[i]) * (distances[i+1] - distances[i]) + distances[i])
    
    return distance

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

def get_object_position(position,angle,sensordist):
    #Determines the coordinates of the box
    x = position[0]
    z = position[1]
    return [x+(sensordist+0.025) * np.sin(angle),z+(sensordist+0.025) * np.cos(angle)]#Adds to centroid, may need to be tuned

def checkIfBox():
    turn()
    if(dist_bottom < 0.8 and np.abs(dist_bottom-dist_top) > 0.04):
        
        

def moveToPosition(position):
    #moves wheels to position (in rads)
    left_wheel.setPosition(position)
    right_wheel.setPosition(position)

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
    
def align(direction):
    currentAngle = compassAngle()
    print(currentAngle)
    if currentAngle != direction:
        turn(direction-currentAngle)
        robot.step(1)
    else:
        print('aligned!')
        pass
        
def passive_wait(time):
    start_time = robot.getTime()
    while start_time + time > robot.getTime():
        print(start_time + time)
        print(robot.getTime())
        robot.step(1)