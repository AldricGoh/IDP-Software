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

def sensor_to_dist(sensor_value:float,bot_length:float,sensor_type)->float:

    if sensor_type == "long":
        lookup = [0.15,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5]
        lookup2 = [2.75,2.51,1.99,1.52,1.25,1.04,0.87,0.79,0.74,0.69,0.6,0.55,0.5,0.47,0.45]
        if sensor_value <= 0.45:
            return 1.5 + bot_length
        index = [ n for n,i in enumerate(lookup2) if i<sensor_value ][0] - 1
        return (sensor_value - lookup2[index]) / (lookup2[index+1] - lookup2[index]) * (lookup[index+1] - lookup[index]) + lookup[index] + bot_length
    else:
        lookup = [0.05,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8]
        lookup2 = [3.1,2.26,1.27,0.92,0.75,0.6,0.5,0.45,0.41]
        if sensor_value <= 0.41:
            return 0.8 + bot_length
        index = [ n for n,i in enumerate(lookup2) if i<sensor_value ][0] - 1
        return (sensor_value - lookup2[index]) / (lookup2[index+1] - lookup2[index]) * (lookup[index+1] - lookup[index]) + lookup[index] + bot_length

def anomalies(sensorLong,sensorShort): 
    if abs(sensorLong - sensorShort) > 0.1:
        #print("Long: " + str(sensorLong))
        #print("Short: " + str(sensorShort))
        return True

def dist(x1:float,z1:float,x2:float,z2:float)->bool:
    return (x2-x1)**2+(x2-x1)**2

def openDoor():
    left_door.setPosition(1.57)
    right_door.setPosition(1.57)

def closeDoor():
    left_door.setPosition(0)
    right_door.setPosition(0)
    
def setSpeed(speedL,speedR):
    
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(speedL*MAX_SPEED)
    right_wheel.setVelocity(speedR*MAX_SPEED)


def move_to_coordinate(destination,early_stop):
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
            #print(dist(coord2d[0],coord2d[1],destination[0],destination[1]))
            if dist(coord2d[0],coord2d[1],destination[0],destination[1])<early_stop:
               leftSpeed = 0
               rightSpeed = 0
               setSpeed(leftSpeed,rightSpeed) 
               return 
                
        elif  (test_angle - theta_destination) > np.pi:
                #print("Turning Left")
            
            leftSpeed = -0.3
            rightSpeed = 0.3
            if  (test_angle - theta_destination) <  2*np.pi - 0.2:
                leftSpeed = -0.8
                rightSpeed = 0.8
            
        elif (test_angle - theta_destination) <= np.pi:
           #print("Turning right")
            leftSpeed = 0.3
            rightSpeed = -0.3
            if  (test_angle - theta_destination) >  0.2:
                leftSpeed = 0.8
                rightSpeed = -0.8
        #print("STEP")       
        setSpeed(leftSpeed,rightSpeed)       
        robot.step(1)
    
def move_to_angle(target):
    while True:
        leftSpeed = 0
        rightSpeed = 0
        angle = convert_compass_angle(compass.getValues())
        theta_destination = target
        if theta_destination <=0:
            theta_destination += 2*np.pi
        test_angle = angle
        
        if test_angle < theta_destination:
            test_angle+=np.pi*2
        if abs(theta_destination - angle) < 0.03:
            leftSpeed = 0
            rightSpeed = 0
            setSpeed(leftSpeed,rightSpeed)   
            return        
        elif  (test_angle - theta_destination) > np.pi:
                #print("Turning Left")
            leftSpeed = -0.3
            rightSpeed = 0.3
            if  (test_angle - theta_destination) <  2*np.pi - 0.2:
                leftSpeed = -0.8
                rightSpeed = 0.8  
        elif (test_angle - theta_destination) <= np.pi:
           #print("Turning right")
            leftSpeed = 0.3
            rightSpeed = -0.3
            if  (test_angle - theta_destination) >  0.2:
                leftSpeed = 0.8
                rightSpeed = -0.8
        #print("STEP")       
        setSpeed(leftSpeed,rightSpeed)       
        robot.step(1)

        
    
    
 
    
def close_scan():
    pass
    
    
def pickup_scanned_box():
    pass

while robot.step(timestep) != -1:
    
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]

    angle = convert_compass_angle(compass.getValues())
    
    move_to_angle(np.pi)

    move_to_angle(3 *np.pi/2)

    """move_to_coordinate([-1,-1],0.1)
    
    move_to_coordinate([0,0],0.05)
    """
    break
    


    pass
    
