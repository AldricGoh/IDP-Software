from controller import Robot, Motor, Keyboard, Compass
import numpy as np
import time


#Useful constants
TIME_STEP = 64
MAX_SPEED = 3

robot = Robot()

#get robot motor devices
left_wheel = robot.getDevice("left_wheel")
right_wheel = robot.getDevice("right_wheel")
left_door = robot.getDevice("left_door")
right_door = robot.getDevice("right_door")
left_wheel.setVelocity(MAX_SPEED)
right_wheel.setVelocity(MAX_SPEED)

#Enable GPS
gps = robot.getDevice("gps")
gps.enable(100)#Sampling period
#Enable compass
compass = robot.getDevice("compass")
compass.enable(100)#Sampling period
#Emitter Receiver

timestep = int(robot.getBasicTimeStep())

def robot_location():
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]
    return coord2d
 
def convert_compass_angle(compass_values:list)->float:

    rad = -np.arctan2(compass_values[0],compass_values[2])
    if rad <=0:
        rad += 2*np.pi
    return rad
"""
def moveToPosition(position):
    #moves wheels to position (in rads)
    left_wheel.setPosition(position)
    right_wheel.setPosition(position)

def move(speed):
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(speed*MAX_SPEED)
    right_wheel.setVelocity(speed*MAX_SPEED)

def turn():
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
    

def checkObstacles():
    #TURN +-66 degrees about axis of symmetry of robot
    #Scan if there are any obstacles to the location desired
    #Using ds_bottom
    #If no, return False
    #If yes, return True
    turnRadian((11*np.pi)/30)
    
def passive_wait(time):
    start_time = robot.getTime()
    while start_time + time > robot.getTime():
        print(start_time + time)
        print(robot.getTime())
        robot.step(1)
        
        
"""
def dist(x1:float,z1:float,x2:float,z2:float)->bool:
    return (x2-x1)**2+(x2-x1)**2


def setSpeed(speedL,speedR):
    
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(speedL*MAX_SPEED)
    right_wheel.setVelocity(speedR*MAX_SPEED)


def move_to_coordinate(destination):
    angle_right = 0 
    distance_right = 0
    while True:
        coord3d = gps.getValues()
        coord2d = [coord3d[0],coord3d[2]]
        leftSpeed = 0
        rightSpeed = 0
        angle = convert_compass_angle(compass.getValues())

        theta_destination = -np.pi/2-np.arctan2(coord2d[1]-destination[1],coord2d[0]-destination[0])
        if theta_destination <=0:
            theta_destination += 2*np.pi
        
        test_angle = angle
    
                
        if test_angle < theta_destination:
            test_angle+=np.pi*2
        if abs(theta_destination - angle) < 0.1:
            leftSpeed = -1
            rightSpeed = -1
            print(dist(coord2d[0],coord2d[1],destination[0],destination[1]))
            if dist(coord2d[0],coord2d[1],destination[0],destination[1])<0.1:
               leftSpeed = 0
               rightSpeed = 0
               setSpeed(leftSpeed,rightSpeed) 
               return 
                
        elif  (test_angle - theta_destination) > np.pi:
                #print("Turning Left")
            leftSpeed = -0.3
            rightSpeed = 0.3
        elif (test_angle - theta_destination) <= np.pi:
           #print("Turning right")
            leftSpeed = 0.3
            rightSpeed = -0.3
        #print("STEP")       
        setSpeed(leftSpeed,rightSpeed)       
        robot.step(1)
    

while robot.step(timestep) != -1:
    
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]

    angle = convert_compass_angle(compass.getValues())
    move_to_coordinate([0.7,0.7])
    
    move_to_coordinate([0,0])
    
    break
    


    pass
    
