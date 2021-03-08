from controller import Robot, Motor, Keyboard, Compass, GPS
import numpy as np
import time
# from functions import *
from setup import *
from classes import *

# Variables
status = Status()
position = [0,0]
true_heading = 0
dist_bottom = 0
dist_top = 0
colors = {"red":False, "green":False}
boxes = []
initial_heading = 0

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
    
def get_object_position(dist, angle, robot_position):
    """Calculate position estimate for object from distance sensor reading and robot orientation and position"""
    dist = dist + DISTANCE_SENSOR_OFFSET
    position = [robot_position[0] + dist*np.cos(angle),
                robot_position[1] + dist*np.sin(angle)]
    return position

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

def update_state():
    """Update all global variables according to the current state of the robot"""
    
    # position
    coord3d = gps.getValues()
    position = [coord3d[0],coord3d[2]]
    
    # heading (in radians 0-2pi)
    compass_raw = compass.getValues()
    heading = round(-np.arctan2(compass_raw[0],compass_raw[2]), 3)
    if(heading <= 0):
        heading += 2*np.pi
    # keep track of revolutions to get true heading that doesnt wrap around   
    spin_direction = np.sign(status.turning_speed)
    if((spin_direction == 1 and (heading < np.pi and status.previous_heading > np.pi)) or (spin_direction == -1 and (heading > np.pi and status.previous_heading < np.pi))):
        status.revolutions += spin_direction
    status.previous_heading = heading
    true_heading = heading + status.revolutions*2*np.pi
    
    # dist_bottom and dist_top not in meters for now
    dist_bottom = longDSReading()
    dist_top = shortDSReading()
    
    # colors
    colors["red"] = (ls_red.getValue() >= 2.5)
    colors["green"] = (ls_green.getValue() >= 2.5)
    
    return position, true_heading, dist_bottom, dist_top, colors

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
    currentAngle = true_heading
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

print('start')

timestep = int(robot.getBasicTimeStep())

position, true_heading, dist_bottom, dist_top, colors = update_state()

while(np.isnan(true_heading)):
    robot.step(1)
    position, true_heading, dist_bottom, dist_top, colors = update_state()   
print("Sensor readings valid")

initial_heading = true_heading
status.scanning == True

while robot.step(timestep) != -1:
    position, true_heading, dist_bottom, dist_top, colors = update_state()
    
    if(status.scanning == True):
        if true_heading == initial_heading:
            status.stop_scan()
            print('here')
        status.start_scan(true_heading)
        if(dist_bottom < 0.8 and np.abs(dist_bottom-dist_top) > 0.04):
            print(get_object_position(dist_bottom, true_heading, position))

        # if(np.abs(true_heading - status.scan.initial_heading) >= 2*np.pi):
            # boxes = evaluate_scan(status.scan.dists_bottom, status.scan.dists_top, status.scan.angles, status.scan.positions)
            # print(boxes)
            # status.start_idle()
        # else:
            # status.scan.dists_bottom.append(dist_bottom)
            # status.scan.dists_top.append(dist_top)
            # status.scan.angles.append(true_heading)
            # status.scan.positions.append(position)
      
        
    
    pass
