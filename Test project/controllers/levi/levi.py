from controller import Robot, Motor, Compass
import numpy as np

from classes import *
from constants import *
from setup import *


# Variables
status = Status()
position = [0,0]
true_heading = 0
dist_bottom = 0
dist_top = 0
boxes = None
color = None


# Functions
def dist(pos, position):
    x1 = pos[0]
    z1 = pos[1]
    x2 = position[0]
    z2 = position[1]
    return (x2-x1)**2+(z2-z1)**2
    
def angle(pos, position):
    x1 = pos[0]
    z1 = pos[1]
    x2 = position[0]
    z2 = position[1]
    direction = -np.arctan2(x1-x2,z1-z2)
    if(direction <= 0):
        direction += 2*np.pi
        direction = round(direction, 2)
        
    return direction

def closest_box():
    answer = []
    for box in boxes:
        if answer == []:
            answer = box
        elif dist(answer, position) > dist(box, position):
            answer = box
        else: pass
    return answer

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
    position = [round(coord3d[0], 2),round(coord3d[2], 2)]
    
    # heading (in radians 0-2pi)
    compass_raw = compass.getValues()
    heading = -np.arctan2(compass_raw[0],compass_raw[2])
    if(heading <= 0):
        heading += 2*np.pi
        heading = round(heading, 2)
    # keep track of revolutions to get true heading that doesnt wrap around   
    spin_direction = np.sign(status.turning_speed)
    if((spin_direction == 1 and (heading < np.pi and status.previous_heading > np.pi)) or (spin_direction == -1 and (heading > np.pi and status.previous_heading < np.pi))):
        status.revolutions += spin_direction
    status.previous_heading = heading
    true_heading = heading + status.revolutions*2*np.pi
    
    # dist_bottom and dist_top not in meters for now
    dist_bottom = long_to_ms(ds_bottom.getValue())
    dist_top = short_to_ms(ds_top.getValue())
    
    # color
    if(ls_red.getValue() >= 2.5):
        color = "red"
    elif(ls_green.getValue() >= 2.5):
        color = "green"
    else:
        color = None
    
    return position, true_heading, dist_bottom, dist_top, color
    

    
<<<<<<< HEAD



=======
def evaluate_scan():
    confirm_boxes = []
    # Initial detection runincluding positions for all detections
    # Group closeby coordinates together that presumably correcpond to the same box
    for item in boxes:
       if boxes.count(item) <= 2:
           boxes.remove(item)
       else:
           if confirm_boxes.count(item) == 0:
               confirm_boxes.append(item)
               boxes.remove(item)
           else:
               boxes.remove(item)
    return confirm_boxes
>>>>>>> 8fa7102cecf59e151b8176f4bb600e4538355b7d

def align(direction):
    if true_heading != direction:
        status.turn((direction-true_heading)/3)
        robot.step(1)
    else:
        print('aligned!')
        status.start_idle()

# Main code body
# Pre update robot state
position, true_heading, dist_bottom, dist_top, color = update_state()

# Wait for startup (compass gave some nan value on the first 2-3 timesteps)
while(np.isnan(true_heading)):
    robot.step(1)
    position, true_heading, dist_bottom, dist_top, color = update_state()   
print("Sensor readings valid")


status.start_scan(true_heading)
#status.turn(0.3)

timestep = int(robot.getBasicTimeStep())  
while robot.step(timestep) != -1:
    # Update robot state
    position, true_heading, dist_bottom, dist_top, color = update_state()
    
    # If in scanning state
    if(status.scanning == True):
        #If scan is done
        if(np.abs(true_heading - status.scan.initial_heading) >= 2*np.pi):
<<<<<<< HEAD
            boxes = status.scan.evaluate_scan()
=======
            status.start_moving()
            boxes = evaluate_scan()
>>>>>>> 8fa7102cecf59e151b8176f4bb600e4538355b7d
            print(boxes)
            status.start_idle()
            
        # If scan is still in progress
        else:
<<<<<<< HEAD
            status.scan.dists_bottom.append(dist_bottom)
            status.scan.dists_top.append(dist_top)
            status.scan.angles.append(true_heading)
            status.scan.positions.append(position)
 
 
 
=======
            if(dist_bottom < 0.8 and np.abs(dist_bottom-dist_top) > 0.04):
                print(get_object_position(dist_bottom, true_heading, position))
                boxes.append(get_object_position(dist_bottom, true_heading, position))
            # status.scan.dists_bottom.append(dist_bottom)
            # status.scan.dists_top.append(dist_top)
            # status.scan.angles.append(true_heading)
            # status.scan.positions.append(position)
    
    if status.moving_to_box == True:
        current_target = closest_box()
        direction = angle(current_target, position)
        if abs(direction-true_heading) > 0.1: align(direction)
        else:
            status.turn(0.3)
            if dist_bottom < 0.8 and np.abs(dist_bottom-dist_top) > 0.04:
                print('Aligned')
                status.start_idle()

>>>>>>> 8fa7102cecf59e151b8176f4bb600e4538355b7d
 
    #print("State: x={:.2f}; y={:.2f}; heading={:.2f}; distance_top={:.6f}; distance_bottom={:.6f}; color={}".format(position[0], position[1], true_heading/(2*np.pi)*360, dist_top, dist_bottom, color))
    pass