from controller import Robot,GPS,Compass,Receiver,Emitter
import numpy as np
import struct
import time
import sys

"""
Setup
"""
robot = Robot()
robot_colour = "green"
home = [0,-0.4]
if robot_colour =="red":
    home = [0,0.4]

start = robot.getTime()
blockid = 0
blocks = []#format: [coord1,coord2,colour,sorted?]
other_robot_coords = [0,0.4]
other_destination = None
current_block = None
block_hitbox_radius = 0.1 #0.056
robot_hitbox_radius = None
robot_status = "scan"
destination = [0.5,0.5]
MAX_SPEED = 3.14
sensorX = 0.115
sensorZ = 0 #redundant
navigation_status = 0
WHEEL_RADIUS = 0.0508
ROBOT_WIDTH = 0.18 + 2*WHEEL_RADIUS
TURN_RADIUS = (ROBOT_WIDTH-WHEEL_RADIUS+1.11)/2
moveWaypoints = [[-0.9, 0.7], [-0.9, -0.7], [0.9, -0.7], \
                    [0.9, 0.7], [0, 0], [0, -0.7], [0, 0.7]]
scanWaypoints = [[-0.9, 0.7], [-0.9, -0.7], [0.9, -0.7], [0.9, 0.7]]
blocksCollected = 0

TIME_STEP = 64

    
#get robot motor devices
left_wheel = robot.getDevice("left_wheel")
right_wheel = robot.getDevice("right_wheel")
left_door = robot.getDevice("left_door")
right_door = robot.getDevice("right_door")
left_wheel.setVelocity(MAX_SPEED)
right_wheel.setVelocity(MAX_SPEED)
    
#get robot sensor devices
ds_right = robot.getDevice('ds_long')
ds_left= robot.getDevice('ds_short')
ls_red = robot.getDevice('ls_red')
ls_green = robot.getDevice('ls_green')

#enable sensors
ds_right.enable(TIME_STEP)
ds_left.enable(TIME_STEP)
ls_red.enable(TIME_STEP) 
ls_green.enable(TIME_STEP)

#Enable GPS
gps = robot.getDevice("gps")
gps.enable(16)#Sampling period
#Enable compass
compass = robot.getDevice("compass")
compass.enable(16)#Sampling period
#Emitter Receiver
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")

emitter.setChannel(1)
receiver.setChannel(2)

if robot_colour == "red":
    emitter.setChannel(2)
    receiver.setChannel(1)

receiver.enable(100)


timestep = int(robot.getBasicTimeStep())



def moveToPosition(position):
    #moves wheels to position (in rads)
    left_wheel.setPosition(position)
    right_wheel.setPosition(position)

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

def convert_compass_angle(compass_values):

    
    rad = -np.arctan2(compass_values[0],compass_values[2])
    if rad <=0:
        rad += 2*np.pi
    return rad

def dist_to_wall(angle,position,sensor_type):
    #Finds what the distance sensor should be reading
    if sensor_type == "short":
        cap = 0.8+sensorX
    else:
        cap = 1.5+sensorX
    x = position[0]
    z = position[1]
    #print("X: " + str(x) + " Z: " + str(z) + " Angle: " + str(angle))
    wallN = abs((1.2-0.01 - x) / np.sin(angle))#0.01 terms as wall has a thickness
    wallS = abs((-1.2+0.01 - x) / np.sin(angle))
    wallW = abs((-1.2+0.01 - z) / np.cos(angle))
    wallE = abs((+1.2-0.01 - z) / np.cos(angle))
    if (angle>=0 and angle <= np.pi/2):
        return min([wallN,wallE,cap])
    elif (angle>=np.pi/2 and angle <= np.pi):
        return min([wallW,wallN,cap])
    if (angle>=np.pi and angle <= 3*np.pi/2):
        return min([wallS,wallW,cap])
    elif (angle>=3*np.pi/2 and angle <= 2*np.pi):
        return min([wallE,wallS,cap])
    else:
        return 0.1

def get_object_position(position,angle,sensordist):
    #Determines the coordinates of the box
    x = position[0]
    z = position[1]
    return [x+(sensordist+0.025) * np.sin(angle),z+(sensordist+0.025) * np.cos(angle)]#Adds to centroid, may need to be tuned     
        
def sensor_to_dist(sensor_value,bot_length,sensor_type):
    #Uses sensitivity curve to calculate actual distance, factors in position of sensor
    #return (1000-sensor_value)/666.7 + bot_length
    
    #Linear interpolation bewteen 2 points
    #print("Value used: " +str(sensor_value))
    if sensor_type == "long":
        lookup = [0.15,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5]
        lookup2 = [2.75,2.51,1.99,1.52,1.25,1.04,0.87,0.79,0.74,0.69,0.6,0.55,0.5,0.47,0.45]
        if sensor_value <= 0.45:
            return 1.5 +sensorX
        index = [ n for n,i in enumerate(lookup2) if i<sensor_value ][0] - 1
        return (sensor_value - lookup2[index]) / (lookup2[index+1] - lookup2[index]) * (lookup[index+1] - lookup[index]) + lookup[index] + bot_length
    else:
        lookup = [0.05,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8]
        lookup2 = [3.1,2.26,1.27,0.92,0.75,0.6,0.5,0.45,0.41]
        if sensor_value <= 0.41:
            return 0.8+sensorX
        index = [ n for n,i in enumerate(lookup2) if i<sensor_value ][0] - 1
        return (sensor_value - lookup2[index]) / (lookup2[index+1] - lookup2[index]) * (lookup[index+1] - lookup[index]) + lookup[index] + bot_length
    
    
def what_is_it(position,other_robot_position):
    #Determines if the box is interesting
    x = position[0]
    z = position[1]  
    for block in blocks:
        if (hitboxcollision(x,z,block[0],block[1],block_hitbox_radius) == True) and (block[3] != "Sorted"):
            #print("This is a dupe, ignore it")
            return "ExistingBox"

    #if hitboxcollision(x,z,other_robot_coords[0],other_robot_coords[1],robot_hitbox_radius) == True:  
    if (x <= 0.2 and x >= -0.2 and ((z <=0.6 and z >= 0.2) or (z >=-0.6 and z <= -0.2))):
        #print("This box has already been sorted")
        return "SortedBox"
    else:
        #print("X: " + str(x) + "Z: " + str(z))
        return "NewBox"
        
        
def hitboxcollision(x1,z1,x2,z2,r2):
    if (x2-x1)**2+(z2-z1)**2 <= r2**2:
        return True
    else:
        return False

        
def dist(x1,z1,x2,z2):
    return ((x2-x1)**2+(z2-z1)**2)*0.5

    
def passive_wait(time):
    start_time = robot.getTime()
    while start_time + time > robot.getTime():
        robot.step(1)
        
        
def message_encode(message_type,content):
    #Encodees message in format (type, coord1, coord2, block id, block action)
    format = "HffH"
    if message_type == "WhereAreYou":
        message = struct.pack(format,1,0,0,0)#Just a ping
    elif message_type == "IAmHere":
        message = struct.pack(format,2,content[0],content[1],0)#coord1, coord2, Null
    elif message_type == "NewBlock":
        message = struct.pack(format,3,content[0],content[1],0)#coord1, coord2, Null
    elif message_type == "BlockRed":
        message = struct.pack(format,4,0,0,content[0])#Null, Null, ID
    elif message_type == "BlockGreen":
        message = struct.pack(format,5,0,0,content[0])#Null, Null, ID
    elif message_type == "MyBlock":
        message = struct.pack(format,6,0,0,content[0])#Null, Null, ID
    elif message_type == "IAmGoingTo":
        message = struct.pack(format,7,content[0],content[1], 0)#coord1, coord2, Null   
    return message

    
def message_decode(message):
    #Decodes the message sent
    data=struct.unpack("HffH",message)
    if data[0] == 1:
        return ("WhereAreYou", [])
    elif data[0] ==2:
        return ("IAmHere", [data[1],data[2]])
    elif data[0] ==3:
        return ("NewBlock", [data[1],data[2],data[3]]) 
    elif data[0] ==4:
        return ("BlockRed", [data[3]])
    elif data[0] ==5:
        return ("BlockGreen", [data[3]])
    elif data[0] ==6:
        return ("MyBlock", [data[3]])
    elif data[0] ==7:
        return ("IAmGoingTo", [data[1],data[2]])
    #Might need a block removed


def sort_all_messages():#Might not need
    while receiver.getQueueLength() > 0:
        message = receiver.getData()
        message_type,data = message_decode(message)
        if message_type == "WhereAreYou":
            pass
        elif message_type == "IAmHere":
            other_robot_coords = data
        elif message_type == "NewBlock":
            blocks.append([data[0],data[1],"Unknown","Unsorted"])
        elif message_type == "BlockRed":
            blocks[data[0]][2] = "red"
        elif message_type == "BlockGreen":
            blocks[data[0]][2] = "green"
        elif message_type == "MyBlock":#Choping block means that it will be the next waypoint
            #for item in blocks:
                #if  item[3] == "Bot2Chope":
                    #item[3] == "Unsorted"
            blocks[data[0]][3] = "Bot2Chope"
            #Also unchope other blocks
        elif message_type == "IAmGoingTo":
            other_destination = [data[0],data[1]]
        receiver.nextPacket()
        #print(blocks)


def scan(sensordist,walldist):
    if sensordist < 1.55 and sensordist<walldist * 0.9:#Need to tune this value so no false positives
        #print("Object found at " + str(angle) + " Sensor dist: "+str(sensordist)+ " Wall dist: " + str(walldist))
        object_position = get_object_position(coord2d,angle,sensordist)
        status = what_is_it(object_position,1)
        if status == "NewBox":
            print("New block")
            
            blocks.append([object_position[0],object_position[1],"Unknown","Unsorted"])
            print(blocks)
            message = message_encode("NewBlock",[object_position[0],object_position[1]])
            emitter.send(message)
 
        else:
            pass

            
def anomalies(sensorLong,sensorShort): 
    
    #print(min([sensorLong,0.8 + sensorX]))
    #print(sensorShort)
    if abs(min([sensorLong,0.8 + sensorX]) - sensorShort) > 0.1:
        #print("anomaly detected")
        #print("Long: " + str(sensorLong))
        #print("Short: " + str(sensorShort))
        return True
        
    else:
        return False


def wheel_travel_info(position, destination):
    """Returns a list of coordinates of the edge of the wheels of the robot
    and their respective destinations"""        
    angle = -np.pi/2-np.arctan2(position[1]-destination[1],position[0]-destination[0])
    if angle <=0:
        angle += 2*np.pi
    
    r = ROBOT_WIDTH/2
    return [[position[0] - r*np.sin(angle), position[1] + r*np.cos(angle)], \
        [position[0] + r*np.sin(angle), position[1] - r*np.cos(angle)], \
        [destination[0] - r*np.sin(angle), destination[1] + r*np.cos(angle)], \
        [destination[0] + r*np.sin(angle), destination[1] - r*np.cos(angle)]] 
  
        
def lines_intersect(pos1, pos2, pos3, pos4):
    """determines whether or not lines
    pos1-pos2 and pos3-pos4 intersect. Returns True if collision occurs"""
    uA = ((pos4[0]-pos3[0])*(pos1[1]-pos3[1]) \
        - (pos4[1]-pos3[1])*(pos1[0]-pos3[0])) / ((pos4[1]-pos3[1])*(pos2[0]-pos1[0]) \
        - (pos4[0]-pos3[0])*(pos2[1]-pos1[1]))
    
    uB = ((pos2[0]-pos1[0])*(pos1[1]-pos3[1]) \
        - (pos2[1]-pos1[1])*(pos1[0]-pos3[0])) / ((pos4[1]-pos3[1])*(pos2[0]-pos1[0]) \
        - (pos4[0]-pos3[0])*(pos2[1]-pos1[1]))
    
    if uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1:
        return True
    else:
        return False
     
def intersect_endzone(current_position, destination):
    """Return True if robot will pass through endzones"""
    #Corners of endzones
    endzone_red = [[0.2, 0.2], [0.2, 0.6], [-0.2, 0.2], [-0.2, 0.6]]
    endzone_green = [[0.2, -0.2], [0.2, -0.6], [-0.2, -0.2], [-0.2, -0.6]]
    
    #Takes into account width of robot
    info = wheel_travel_info(current_position, destination)
    
    for i in range(2):
        if lines_intersect(info[i], info[i+2], endzone_red[0], endzone_red[1]) \
            or lines_intersect(info[i], info[i+2], endzone_red[0], endzone_red[2]) \
            or lines_intersect(info[i], info[i+2], endzone_red[1], endzone_red[3]) \
            or lines_intersect(info[i], info[i+2], endzone_red[2], endzone_red[3]) \
            or lines_intersect(info[i], info[i+2], endzone_green[0], endzone_green[1]) \
            or lines_intersect(info[i], info[i+2], endzone_green[0], endzone_green[2]) \
            or lines_intersect(info[i], info[i+2], endzone_green[1], endzone_green[3]) \
            or lines_intersect(info[i], info[i+2], endzone_green[2], endzone_green[3]):
                
                return True
    
    else:
        
        return False
        
#Can make an edit to this function to avoid redundant calculations
def intersect_other_robot_path(current_position, destination, other_position, other_destination):
    """Returns True if robots paths coincide""" 
    #Takes into account width of robot
    info_1 = wheel_travel_info(current_position, destination)
    info_2 = wheel_travel_info(current_position, destination)
    
    for i in range(2):
        for j in range(2):
            if lines_intersect(info_1[i], info_1[i+2], info_2[j], info_2[j+2]):
                return True
    
    return False   
     
        
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
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        sensordistShort = sensor_to_dist(ds_left.getValue(),sensorX,"short") 
        if sensordistShort  < 0.2:
            print("obstacle")
            setSpeed(1,1)
        robot.step(1)
    
def move_to_angle(target):
    #print("Moving to angle")
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
        if abs(theta_destination - angle) < 0.01:
            leftSpeed = 0
            rightSpeed = 0
            setSpeed(leftSpeed,rightSpeed)   
            return        
        elif  (test_angle - theta_destination) > np.pi:
                #print("Turning Left")
            leftSpeed = -0.1
            rightSpeed = 0.1
            if  (test_angle - theta_destination) <  2*np.pi - 0.2:
                leftSpeed = -0.8
                rightSpeed = 0.8  
        elif (test_angle - theta_destination) <= np.pi:
           #print("Turning right")
            leftSpeed = 0.1
            rightSpeed = -0.1
            if  (test_angle - theta_destination) >  0.2:
                leftSpeed = 0.8
                rightSpeed = -0.8
        #print("STEP")       
        setSpeed(leftSpeed,rightSpeed)       
        robot.step(1)
    

def short_scan():
    move_to_coordinate(destination,0.05)
    angle = convert_compass_angle(compass.getValues())
    lowest = 100
    lowest_angle = None
    start = robot.getTime()
    leftSpeed = -0.2
    rightSpeed = 0.2
    setSpeed(leftSpeed,rightSpeed)  
    
    
    
    #Find angle bit method 1
    """
    while True:
        
        current_angle = convert_compass_angle(compass.getValues())
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        sensordistShort = sensor_to_dist(ds_left.getValue(),sensorX,"short")
        coord3d = gps.getValues()
        coord2d = [coord3d[0],coord3d[2]]

        if anomalies(sensordistLong,sensordistShort):
            object_coord = get_object_position(coord2d,current_angle,sensordistLong)
            
            if dist(blocks[blockid][0],blocks[blockid][1],object_coord[0],object_coord[1]) < 0.05:
                if sensordistLong < lowest:
                    lowest = sensordistLong
                    lowest_angle = current_angle
                    #print(current_angle)
                    #print(sensordistLong)
        if abs(angle - current_angle) < 0.1 and time.time()-start > 2:
            break
        
        robot.step(1)
    """
    move_to_angle(angle-0.5)

    leftSpeed = -0.2
    rightSpeed = 0.2
    setSpeed(leftSpeed,rightSpeed)  
    seen_yet = 0
    while True:
        endThisSuffering()
        current_angle = convert_compass_angle(compass.getValues())
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        sensordistShort = sensor_to_dist(ds_left.getValue(),sensorX,"short")
        coord3d = gps.getValues()
        coord2d = [coord3d[0],coord3d[2]]
        if seen_yet == 0:
            if anomalies(sensordistLong,sensordistShort):
                #print("seen anomaly")
                object_coord = get_object_position(coord2d,current_angle,sensordistLong)
                if dist(blocks[blockid][0],blocks[blockid][1],object_coord[0],object_coord[1]) < 0.05:
                   seen_yet = 1
                   angle1 = current_angle
        else:
            #(anomalies(sensordistLong,sensordistShort))
            if anomalies(sensordistLong,sensordistShort) == False:
                #print("No anomaly")
                angle2 = current_angle
                if angle2 < angle1:
                    angle1-=np.pi*2
                lowest_angle = (angle1+angle2)/2
                break
        
        
        robot.step(1)
   
        
    #print(lowest_angle)
    move_to_angle(lowest_angle)
    #print(convert_compass_angle(compass.getValues()))
    leftSpeed = -0.3
    rightSpeed = -0.3
    setSpeed(leftSpeed,rightSpeed)  
    while True:
        endThisSuffering()
        ls_red_value = ls_red.getValue()
        ls_green_value = ls_green.getValue()
    
        if ls_red_value > 0:
            if robot_colour == "green":
                print("Oh no a red block")
                blocks[blockid][2] = "red"
                blocks[blockid][3] = "ignore"
                message = message_encode("BlockRed",[blockid])
                emitter.send(message)
                return 
            else:
                message = message_encode("BlockRed",[blockid])
                emitter.send(message)
                blocks[blockid][2] = "red"
                #blocks[blockid][3] = "Sorted"
                break
        if ls_green_value > 0:
            if robot_colour == "red":
                print("Oh no a green block")
                message = message_encode("BlockGreen",[blockid])
                emitter.send(message)
                blocks[blockid][2] = "green"
                blocks[blockid][3] = "ignore"
                return 
            else:
                message = message_encode("BlockGreen",[blockid])
                emitter.send(message)
                blocks[blockid][2] = "green"
                #blocks[blockid][3] = "Sorted"
                break
            
        robot.step(1)
    
    target_angle = lowest_angle + np.pi
    if target_angle >= 2 *np.pi:
        target_angle -= 2*np.pi
    move_to_angle(target_angle)
    openDoor()
    start = robot.getTime()
    
    drive_straight(0.5,0.5,2)
    closeDoor()
    move_to_coordinate(home,0.02)
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]
    #print(coord2d[0])
    #print(coord2d[1])
    #print(home[0])
    #print(home[1])
    end_angle = np.arctan2(coord2d[0]-home[0],coord2d[1]-home[1])
    
    move_to_angle(end_angle)
    openDoor()
    drive_straight(-0.5,-0.5,3)
    closeDoor()
    blocks[blockid][3] = "Sorted"
    return 

def drive_straight(leftSpeed,rightSpeed,t):
    start = robot.getTime()
    while robot.getTime()-start <t:
        endThisSuffering()
        setSpeed(leftSpeed,rightSpeed)  
        robot.step(1)
 
def intersect_protocol(destination):
    """Function updates variables in case of predicted collision
    If collision is expected, returns False and if not True
    green bot is given priority so returns True in case of expected robot collision"""

    if abs(coord2d[0]) <= 0.2 and (abs(coord2d[1]) <= 0.6 and abs(coord2d[1] >= 0.2)):
        #Robot position is in endzone, proceed normally
        pass
            
    else: 
        if intersect_endzone(coord2d, destination):
            #endzone will be breached
            #Change destination (need to figure out how)
            #Choose nearest waypoint (moveWaypoints)

            nearest_waypoint_distance = dist(coord2d[0],coord2d[1],moveWaypoints[0][0],moveWaypoints[0][1])   
            destination = moveWaypoints[0]
            for waypoint in moveWaypoints:
                if nearest_waypoint_distance > dist(coord2d[0],coord2d[1],waypoint[0],waypoint[1]):

                    if coord2d != waypoint:
                        destination = waypoint
                        nearest_waypoint_distance = dist(coord2d[0],coord2d[1],waypoint[0],waypoint[1])

                    else:
                        pass       
            robot_status = "navigation_move"

            return False
            
    #Check if paths of bots will intersect
    #If intersect, let green go first until paths do not intersect
    if intersect_other_robot_path(coord2d, destination, other_robot_coords, other_destination):
        #REMEMBER TO CHANGE THE VARIABLES
        if robot_colour == 'red':
            theta_destination = -np.pi/2-np.arctan2(coord2d[1]-destination[1],coord2d[0]-destination[0])
            if theta_destination <=0:
                theta_destination += 2*np.pi
            
            move_to_angle(theta_destination)
            #Reverse robot length
            drive_straight(-1, -1,1)
            return False
            
        else:
            
            #for green bot just proceed as usual
            return True
    
    return True
       
def endThisSuffering():
    if robot.getTime() - start > 293:
        move_to_coordinate(home, 0.01)
        print("I'm back home!")
        sys.exit()

    
destination = [0, 0.7]

while robot.step(TIME_STEP) != -1:
    
    endThisSuffering()
       
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]

    angle = convert_compass_angle(compass.getValues())

    
    if robot_status == 'scan':
        destination = [0, 0.7]

        if abs(coord2d[0]) <= 0.2 and (abs(coord2d[1]) <= 0.6 and abs(coord2d[1] >= 0.2)):
            #Robot position is in endzone, proceed normally
            move_to_coordinate(destination, 0.1)
                
        else: 
            if intersect_endzone(coord2d, destination):
                #endzone will be breached
                #Change destination (need to figure out how)
                #Choose nearest waypoint (moveWaypoints)

                nearest_waypoint_distance = dist(coord2d[0],coord2d[1],moveWaypoints[0][0],moveWaypoints[0][1])   
                destination = moveWaypoints[0]
                for waypoint in moveWaypoints:
                    if nearest_waypoint_distance > dist(coord2d[0],coord2d[1],waypoint[0],waypoint[1]):
                        if destination != waypoint:
                            destination = waypoint
                            nearest_waypoint_distance = dist(coord2d[0],coord2d[1],waypoint[0],waypoint[1])
                            print(coord2d, destination, dist(coord2d[0],coord2d[1],waypoint[0],waypoint[1]))
                            
                        else:
                            pass  
                                 
                robot_status = "navigation_move"
                pass
         
  
    if robot_status == "navigation_move":
        print(destination)
        move_to_coordinate(destination, 0)        
        robot_status = 'exit'
        pass
        
    if robot_status == 'exit':
        print(coord2d)
        sys.exit()

        
        