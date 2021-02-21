from controller import Robot,GPS,Compass,Receiver,Emitter
import numpy as np
import struct

"""
Setup
"""

TIME_STEP = 64
robot = Robot()

#Sensors
ds = []
dsNames = ['ds_front']
for i in range(1):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)
    
#Enable GPS
gps = robot.getDevice("gps")
gps.enable(100)#Sampling period


#Enable compass
compass = robot.getDevice("compass")
compass.enable(100)#Sampling period

#Wheels
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
#Emitter Receiver
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
emitter.setChannel(1)
receiver.setChannel(2)
receiver.enable(100)



blocks = []#format: coord1,coord2,colour,sorted?




def convert_compass_angle(compass_values):
    #Returns angle with positive X being 0 degrees
    rad = np.pi/2 - np.arctan2(compass_values[0],compass_values[2])
    #if rad <= 0:
        #rad += 2*np.pi
    
    #rad+= np.pi/2
    if rad <=0:
        rad += 2*np.pi
    return rad
    

def dist_to_wall(angle,position):
    x = position[0]
    z = position[1]
    #0.01 terms as wall has thickness
    wallN = abs((1.2-0.01 - x) / np.sin(angle))
    wallS = abs((-1.2+0.01 - x) / np.sin(angle))
    wallW = abs((-1.2+0.01 - z) / np.cos(angle))
    wallE = abs((+1.2-0.01 - z) / np.cos(angle))
    if (angle>=0 and angle <= np.pi/2):
        #print("North East")
        return min([wallN,wallE])
    elif (angle>=np.pi/2 and angle <= np.pi):
        #print("North West")
        return min([wallW,wallN])
    if (angle>=np.pi and angle <= 3*np.pi/2):
        #print("South West")
        return min([wallS,wallW])
    elif (angle>=3*np.pi/2 and angle <= 2*np.pi):
        #print("South East")
        return min([wallE,wallS])
    else:
          return 1

def get_object_position(position,angle,sensordist):
    x = position[0]
    z = position[1]
    return [x+sensordist * np.sin(angle),z+sensordist * np.cos(angle)]      
        
def sensor_to_dist(sensor_value,bot_length):
    #Uses sensitivity curve to calculate actual distance
    return (1000-sensor_value)/666.7 + bot_length
    
    
def what_is_it(position,other_robot_position):
    #other_robot_position just dummy variable
    x = position[0]
    z = position[1]
    if (x <= 0.2 and x >= -0.2 and ((z <=0.6 and z >= 0.2) or (z >=-0.6 and z <= -0.2))):
        return "SortedBox"
    else:
        return "NewBox"
        
        
def send_message(message_type,content):
    #Message in format (type, coord1, coord2, block id, block action)
    #Messages:
    #Where are you
    #I am here
    #New block scanned
    #Checking out new block
    #Block colour confirmed
    #Taking block back
    #I am done?
    pass
    
def receive_message():
    pass

def sort_all_messages():
    #When it pings for a location it will want to hear back so all outstanding messages are sorted
    pass

        

while robot.step(TIME_STEP) != -1:
    leftSpeed = -1.0
    rightSpeed = 1.0
    
    
    
    
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]
    angle = convert_compass_angle(compass.getValues())



    
    walldist =  dist_to_wall(angle,coord2d)
    sensordist = sensor_to_dist(ds[0].getValue(),0.1)
    
    
    
    if sensordist < 1.55 and sensordist<walldist * 0.9:#Need to tune this value so no false positives
        #print("Object found at " + str(angle) + " Sensor dist: "+str(sensordist)+ " Wall dist: " + str(walldist))
        object_position = get_object_position(coord2d,angle,sensordist)
        status = what_is_it(object_position,1)
        if status == "SortedBox":
            #print("This box has already been put away")
            pass
        else:
            #print("New object found at: " + str(object_position))
            message = struct.pack("b",5)
            emitter.send(message)
 
        
       
    
    

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)
    
    #print(angle)      
    #print("Wall dist: " + str(walldist))
    #print("Sensor dist: " + str(sensordist))