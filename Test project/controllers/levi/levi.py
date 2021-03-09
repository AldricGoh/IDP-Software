import numpy as np

from classes import *
from constants import *
from setup import *


# Variables
status = Status()
position = [0,0]
heading = 0
dist_bottom = 0
dist_top = 0
boxes = None
color = None


# Functions
def long_to_ms(reading):
    """Use sensitivity curve to convert distance sensor raw data to m distance"""
    
    # Sensitivity curve points
    distances = [0.15,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.3,1.5]
    readings = [2.75,2.51,1.99,1.52,1.25,1.04,0.87,0.79,0.74,0.69,0.6,0.55,0.5,0.47,0.45]
    
    distance = distances[0]
    if reading <= readings[-1]:
        distance = distances[-1]
        
    # Return interpolated value
    for i in range(len(readings)-1):
        if readings[i] >= reading and readings[i+1] < reading:
            distance = ((reading - readings[i]) / (readings[i+1] - readings[i]) * (distances[i+1] - distances[i]) + distances[i])
    
    return distance
    
def short_to_ms(reading):
    """Use sensitivity curve to convert distance sensor raw data to m distance"""
    
    # Sensitivity curve points
    distances = [0.05,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.8]
    readings = [3.1,2.26,1.27,0.92,0.75,0.6,0.5,0.45,0.41]
    
    distance = distances[0]
    if reading <= readings[-1]:
        distance = distances[-1]
        
    # Return interpolated value
    for i in range(len(readings)-1):
        if readings[i] >= reading and readings[i+1] < reading:
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
    if heading <= 0:
        heading += 2*np.pi
        heading = round(heading, 2)
    # keep track of revolutions to get true heading that doesnt wrap around   
    spin_direction = np.sign(status.turning_speed)
    if (spin_direction == 1 and (heading < np.pi and status.previous_heading > np.pi)) or (spin_direction == -1 and (heading > np.pi and status.previous_heading < np.pi)):
        status.revolutions += spin_direction
    status.previous_heading = heading
    true_heading = heading + status.revolutions*2*np.pi
    
    # dist_bottom and dist_top not in meters for now
    dist_bottom = long_to_ms(ds_bottom.getValue())
    dist_top = short_to_ms(ds_top.getValue())
    
    # color
    if ls_red.getValue() >= 2.5:
        color = "red"
    elif ls_green.getValue() >= 2.5:
        color = "green"
    else:
        color = None
    
    return position, true_heading, dist_bottom, dist_top, color


# Main code body
# Pre update robot state
position, heading, dist_bottom, dist_top, color = update_state()

# Wait for startup (compass gave some nan value on the first 2-3 timesteps)
while(np.isnan(heading)):
    robot.step(1)
    position, heading, dist_bottom, dist_top, color = update_state()   
print("Sensor readings valid")


status.start_scan(heading)

timestep = int(robot.getBasicTimeStep())  
while robot.step(timestep) != -1:
    # Update robot state
    position, heading, dist_bottom, dist_top, color = update_state()
    
    # If in scanning state
    if status.scanning:
        # If scan is done
        if np.abs(heading - status.scan.initial_heading) >= 2*np.pi:
            boxes = status.scan.evaluate_scan()
            print("Scan finsihed with {} box(es) found at locations {}".format(len(boxes), boxes))
            status.start_align(heading, position, boxes.closest_to_position(position))
            
        # If scan is still in progress
        else:
            status.scan.dists_bottom.append(dist_bottom)
            status.scan.dists_top.append(dist_top)
            status.scan.angles.append(heading)
            status.scan.positions.append(position)
    
    
    if status.aligning:
        spin_direction = np.sign(status.turning_speed)
        # If align is done
        if (spin_direction == -1 and heading < status.align.target_heading) or (spin_direction == 1 and heading > status.align.target_heading):
            status.start_move_to_box(boxes.closest_to_position(position))
            print("Align finished with heading {:.2f}".format(heading%(2*np.pi)/(2*np.pi)*360))
            
        # If align is close to being done and we havent slowed down yet
        elif (np.abs(status.align.target_heading - heading) < 0.1) and (np.abs(status.turning_speed) == ALIGN_SPEED):
            status.turn(spin_direction*FINE_ALIGN_SPEED)
            
            
    if status.moving_to_box:
        print("Color sensor distance to estimated block positions {:.2f}".format(status.move_to_box.distance(position, heading)))
        
        if color!=None:
            print("Color stop")
            status.start_idle()
        elif status.move_to_box.distance(position, heading) < 0.02:
            print("Distance stop")
            status.start_idle()


 
    #print("State: x={:.2f}; y={:.2f}; heading={:.2f}; distance_top={:.6f}; distance_bottom={:.6f}; color={}".format(position[0], position[1], heading/(2*np.pi)*360, dist_top, dist_bottom, color))
    pass