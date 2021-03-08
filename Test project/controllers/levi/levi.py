from controller import Robot, Motor, Compass
import numpy as np

from classes import *
from constants import *
from setup import *
from collections import OrderedDict


# Variables
status = Status()
position = [0,0]
true_heading = 0
dist_bottom = 0
dist_top = 0
boxes = []
colors = {"red":False, "green":False}

# Functions
def long_to_ms(reading):
    """Use sensitivity curve to convert distance sensor raw data to m distance"""
    
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
    
def short_to_ms(reading):
    """Use sensitivity curve to convert distance sensor raw data to m distance"""
    
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
    heading = -np.arctan2(compass_raw[0],compass_raw[2])
    if(heading <= 0):
        heading += 2*np.pi
    # keep track of revolutions to get true heading that doesnt wrap around   
    spin_direction = np.sign(status.turning_speed)
    if((spin_direction == 1 and (heading < np.pi and status.previous_heading > np.pi)) or (spin_direction == -1 and (heading > np.pi and status.previous_heading < np.pi))):
        status.revolutions += spin_direction
    status.previous_heading = heading
    true_heading = heading + status.revolutions*2*np.pi
    
    # dist_bottom and dist_top not in meters for now
    dist_bottom = long_to_ms(ds_bottom.getValue())
    dist_top = short_to_ms(ds_top.getValue())
    
    # colors
    colors["red"] = (ls_red.getValue() >= 2.5)
    colors["green"] = (ls_green.getValue() >= 2.5)
    
    return position, true_heading, dist_bottom, dist_top, colors
    
def get_object_position(dist, angle, robot_position):
    """Calculate position estimate for object from distance sensor reading and robot orientation and position"""
    dist = dist + DISTANCE_SENSOR_OFFSET
    position = [round(robot_position[0] + dist*np.sin(angle), 2),
                round(robot_position[1] + dist*np.cos(angle), 2)]
    return position
    
def evaluate_scan():
    confirm_boxes = []
    # Initial detection runincluding positions for all detections
    for item in boxes:
       if boxes.count(item) <= 2:
           boxes.remove(item)
       else:
           if confirm_boxes.count(item) == 0:
               confirm_boxes.append(item)
               boxes.remove(item)
           else:
               boxes.remove(item)
    # Group closeby coordinates together that presumably correcpond to the same box
    return confirm_boxes

# Main code body
# Pre update robot state
position, true_heading, dist_bottom, dist_top, colors = update_state()

# Wait for startup (compass gave some nan value on the first 2-3 timesteps)
while(np.isnan(true_heading)):
    robot.step(1)
    position, true_heading, dist_bottom, dist_top, colors = update_state()   
print("Sensor readings valid")

status.start_scan(true_heading)
#status.turn(0.3)
timestep = int(robot.getBasicTimeStep())  
while robot.step(timestep) != -1:
    # Update robot state
    position, true_heading, dist_bottom, dist_top, colors = update_state()
    
    if(status.scanning == True):
        if(np.abs(true_heading - status.scan.initial_heading) >= 2*np.pi):
            status.start_idle()
            boxes = evaluate_scan()
            print(boxes)
        else:
            if(dist_bottom < 0.8 and np.abs(dist_bottom-dist_top) > 0.04):
                print(get_object_position(dist_bottom, true_heading, position))
                boxes.append(get_object_position(dist_bottom, true_heading, position))
            # status.scan.dists_bottom.append(dist_bottom)
            # status.scan.dists_top.append(dist_top)
            # status.scan.angles.append(true_heading)
            # status.scan.positions.append(position)
 
    #print("State: x={:.2f}; y={:.2f}; heading={:.2f}; distance_top={:.6f}; distance_bottom={:.6f}; red={}, green={}".format(position[0], position[1], true_heading/(2*np.pi)*360, dist_top, dist_bottom, colors["red"], colors["green"]))
    pass