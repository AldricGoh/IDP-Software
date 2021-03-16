import numpy as np

from classes import *
from constants import *
from setup_green import *


# Variables
# status = Status()
position = [0,0]
heading = 0
dist_bottom = 0
dist_top = 0
boxes = BoxList([[0, 2], [1, 3], [2, 6]])
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

def message_encode(message_type, content):
    #Encodees message in format 
    #(coord1, coord2, block color, Am I going for it?, Is it removed?)
    format = "ff?"
        
    # if content[2] == 'red':
        # content[2] = 1
    # elif content[2] == 'green':
        # content[2] = 2
    # else: pass
        
    if message_type == "MyBlock":
        message = struct.pack(format, content[0],content[1], True)
    elif message_type == "YourBlock":
        message = struct.pack(format, content[0],content[1], False)
    return message

def message_decode(message):
    #Decodes the message sent
    data=list(struct.unpack("ff?",message))
        
    # if data[2] == 1:
        # data[2] = 'red'
    # elif content[2] == 2:
        # content[2] = 'green'
    # else: pass
    
    return data

def check_messages():
    #Checks if we get any messages
    while receiver.getQueueLength() > 0:
        message = receiver.getData()
        message = message_decode(message)
        if message[2] == True: #Other robot is going for it
            #check if box are in boxes list
            #remove if yes
            for box in boxes:
                if [message[0], message[1]] == box or boxes.distance([message[0], message[1]], box) < 0.07:
                    boxes.remove(box)
            
        elif message[2] == False: #Not going for box (must be your colour)
            #add to boxes
            boxes.append([message[0], message[1]])
                
        receiver.nextPacket()    
    
    pass


timestep = int(robot.getBasicTimeStep())  
while robot.step(timestep) != -1:
    # Update robot state & receiver
    check_messages()

    emitter.send(message_encode("MyBlock", [1, 3]))
    emitter.send(message_encode("YourBlock", [1, 10]))
    
    print("GREEN BOXES:" + str(boxes))
        
 
    #print("State: x={:.2f}; y={:.2f}; heading={:.2f}; distance_top={:.6f}; distance_bottom={:.6f}; color={}".format(position[0], position[1], heading/(2*np.pi)*360, dist_top, dist_bottom, color))
    pass