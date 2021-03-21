#Import modules
from controller import Robot,GPS,Compass,Receiver,Emitter
import numpy as np
import struct
import time
import sys

#Note some code commented are because it does not work properly when ran,
#But in theory it should be able to work. Functions or those codes are also
#commented along it
#Some functions are self explanatory

"""
Setup
"""
#Change robot colour variable for the two different robots, either red or green
#This affects several other functions further down so that the 2 controllers are identical except for this one line
robot_colour = "green"
home = [0,-0.4]
scanWaypoints = [[0.7, -0.9], [-0.7, -0.9], [-0.7, 0.9], [0.7, 0.9]]

if robot_colour =="red":
    home = [0,0.4]
    scanWaypoints = [[0.9, -0.7], [0.9, 0.7], [-0.9, 0.7], [-0.9, -0.7]]



#Blocks are stored in a list with the relevant data indexed as below
#Both robots share the same list
blocksCollected = 0
blockid = 0
blocks = []#format: [coord1,coord2,colour,sorted?]


other_robot_coords = [0,0.4]
current_block = None

#Hitblock radii set up so that a block will not be duplicated in the block list
block_hitbox_radius = 0.2 #0.056
robot_hitbox_radius = None


robot_status = "initial scan"
destination = [0.5,0.5]
MAX_SPEED = 3.14
sensorX = 0.115
sensorZ = 0 #redundant
navigation_status = 0
WHEEL_RADIUS = 5.08
ROBOT_WIDTH = 18 + 2*WHEEL_RADIUS
TURN_RADIUS = (ROBOT_WIDTH-WHEEL_RADIUS+1.11)/2
#moveWaypoints are used for a destination to manuever the robots to avoid the endzones
#moveWaypoints = [[-0.7, 0.9], [-0.7, -0.9], [0.7, -0.9], [0.7, 0.9], [0, 0], [-0.7, 0], [0.7, 0], [0, -0.9], [0, 0.9]]
#previousWaypoints = []

TIME_STEP = 64
robot = Robot()
start = robot.getTime()
 
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
gps.enable(100)#Sampling period
#Enable compass
compass = robot.getDevice("compass")
compass.enable(100)#Sampling period
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

def moveToPosition(position):#This function was unused
    """moves wheels to position (in rads)"""
    left_wheel.setPosition(position)
    right_wheel.setPosition(position)

def openDoor():
    left_door.setPosition(1.57)
    right_door.setPosition(1.57)

    
def closeDoor():
    left_door.setPosition(0)
    right_door.setPosition(0)
    
    
def setSpeed(speedL,speedR):
	"""Set the speed for the left and right motors"""
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(speedL*MAX_SPEED)
    right_wheel.setVelocity(speedR*MAX_SPEED)

def convert_compass_angle(compass_values):#Converts the 3 coordinate compass reading into an angle that can be used in other calculations
    rad = -np.arctan2(compass_values[0],compass_values[2])
    if rad <=0:
        rad += 2*np.pi
    return rad

def dist_to_wall(angle,position,sensor_type):
    #Finds what the distance sensor should be reading

    #Each sensor has a maximum reading, this prevents the program expecting the sensors from detecting anything higher
    if sensor_type == "short":
        cap = 0.8+sensorX
    else:
        cap = 1.5+sensorX

        
    x = position[0]
    z = position[1]
    #print("X: " + str(x) + " Z: " + str(z) + " Angle: " + str(angle))
    #Calculates the distance of the robot to each wall
    wallN = abs((1.2-0.01 - x) / np.sin(angle))#0.01 terms as wall has a thickness
    wallS = abs((-1.2+0.01 - x) / np.sin(angle))
    wallW = abs((-1.2+0.01 - z) / np.cos(angle))
    wallE = abs((+1.2-0.01 - z) / np.cos(angle))

    #Selects which two walls the robot is facing so that the two it is facing away from can be ignored
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
    
    #Linear interpolation between 2 points
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
    
def endThisSuffering():
    #This function is inserted in most looping parts of the code so that if the robot gets stuck, it will still return to the start after 280 seconds and then stop
    if robot.getTime() - start > 280:
        move_to_coordinate(home, 0.01)
        print("I'm back home!")
        sys.exit()        
    
    
def what_is_it(position,other_robot_position):
    #Determines if the box is interesting by comparing it with the other blocks that have been found
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
    #Returns if one coordinae lies in the range of another
    if (x2-x1)**2+(z2-z1)**2 <= r2**2:
        return True
    else:
        return False
        
def dist(x1,z1,x2,z2):
    #Returns the distance between 2 points, similar to previous one
    return ((x2-x1)**2+(z2-z1)**2)*0.5


    
def passive_wait(time):
    #Used to make the robot do an instruction for a given amount of time
    #Useful for hard coding pickup and drop off
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
    #Additional type of message for collision avoidance
    # elif message_type == "IAmGoingTo":
    #     message = struct.pack(format,7,content[0],content[1], 0)#coord1, coord2, Null
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
    #Decoding the additional type of message
    # elif data[0] ==7:
    #     return ("IAmGoingTo", [data[1],data[2]])

def sort_all_messages():
    #Sorts out all the messages and does the required actions for each
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
        #Sorting the additional message type
        # elif message_type == "IAmGoingTo":
        #     other_destination = [data[0],data[1]]
        receiver.nextPacket()
        #print(blocks)

def scan(sensordist,walldist):
    #Handles the logic behind if a block has been found or not
    if sensordist < 1.55 and sensordist<walldist * 0.9:#Need to tune this value so no false positives
        #print("Object found at " + str(angle) + " Sensor dist: "+str(sensordist)+ " Wall dist: " + str(walldist))
        object_position = get_object_position(coord2d,angle,sensordist)
        status = what_is_it(object_position,1)
        if status == "NewBox":
            print("A new block has been found at: " +str(object_position))
            
            blocks.append([object_position[0],object_position[1],"Unknown","Unsorted"])
            
            message = message_encode("NewBlock",[object_position[0],object_position[1]])
            emitter.send(message)
 
        else:
            pass
            
def anomalies(sensorLong,sensorShort): 
    #A second method to determine if a block is present
    #This does so by comparing the values of a sensor at block level and a sensor above the blocks
    #print(min([sensorLong,0.8 + sensorX]))
    #print(sensorShort)
    if abs(min([sensorLong,0.8 + sensorX]) - sensorShort) > 0.1:
        #print("anomaly detected")
        #print("Long: " + str(sensorLong))
        #print("Short: " + str(sensorShort))
        return True
        
    else:
        return False

"""This chuck of code was not utilised in the main loop, however was intended to 
given more time to debug it."""        
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

"""End of chunk of unutilised code"""                 



def move_to_coordinate(destination,early_stop):
    #Moves a robot to a fixed coordinate, has an ealry stop so that the robot doesn't try to get absolute precision
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
            
            leftSpeed = -0.4
            rightSpeed = 0.3
            if  (test_angle - theta_destination) <  2*np.pi - 0.2:
                leftSpeed = -0.9
                rightSpeed = 0.8
            
        elif (test_angle - theta_destination) <= np.pi:
           #print("Turning right")
            leftSpeed = 0.3
            rightSpeed = -0.4
            if  (test_angle - theta_destination) >  0.2:
                leftSpeed = 0.8
                rightSpeed = -0.9
        #print("STEP")       
        setSpeed(leftSpeed,rightSpeed)
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        sensordistShort = sensor_to_dist(ds_left.getValue(),sensorX,"short") 
        if sensordistShort  < 0.3:
            print("There is a robot in my way")
            setSpeed(1,1)
            passive_wait(2)
        robot.step(1)
    
def move_to_angle(target):
    #print("Moving to angle")
    #Rotates the robot until it is facing a certain angle
    #Speed decreases as the robot gets clsoerr to the angle so that it doesn't overshoot
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
                leftSpeed = -0.7
                rightSpeed = 0.7  
        elif (test_angle - theta_destination) <= np.pi:
           #print("Turning right")
            leftSpeed = 0.1
            rightSpeed = -0.1
            if  (test_angle - theta_destination) >  0.2:
                leftSpeed = 0.7
                rightSpeed = -0.7
        #print("STEP")       
        setSpeed(leftSpeed,rightSpeed)       
        robot.step(1)
    

def short_scan():
    #This function encompasses the point at which the robot decides it is agong to a specific block all the way to carrying it back to the end zone
    move_to_coordinate(destination,0.05)
    angle = convert_compass_angle(compass.getValues())
    lowest = 100
    lowest_angle = None
    #start = time.time()
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
    loop_preventer = robot.getTime()
    #This part finds the centre of the block by seeing where the two edges are and going to the average angle
    while True:
        endThisSuffering()
        current_angle = convert_compass_angle(compass.getValues())
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        sensordistShort = sensor_to_dist(ds_left.getValue(),sensorX,"short")
        coord3d = gps.getValues()
        coord2d = [coord3d[0],coord3d[2]]
        if robot.getTime() - loop_preventer > 10:
            print("This block is gone goddamnit")
            if robot_colour == "red":
                message = message_encode("BlockRed",[blockid])
                emitter.send(message)
                blocks[blockid][3] = "ignore"
                blocks[blockid][2] = "green"

            else:
                message = message_encode("BlockGreen",[blockid])
                emitter.send(message)
                blocks[blockid][3] = "ignore"
                blocks[blockid][2] = "red"
            return False
        
        
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
    #Reverses until it sees the colour of the block
    #Communicates with the robot and takes a course of action based on what the robots colour is
    while True:
        endThisSuffering()
        ls_red_value = ls_red.getValue()
        ls_green_value = ls_green.getValue()
    
        if ls_red_value > 0:
            if robot_colour == "green":
                print("This block is red but I am green, get out of my sight")
                blocks[blockid][2] = "red"
                blocks[blockid][3] = "ignore"
                message = message_encode("BlockRed",[blockid])
                emitter.send(message)
                return False
            else:
                print("This block is red and I am red, praise be")
                message = message_encode("BlockRed",[blockid])
                emitter.send(message)
                blocks[blockid][2] = "red"
                #blocks[blockid][3] = "Sorted"
                break
        if ls_green_value > 0:
            if robot_colour == "red":
                print("This block is green but I am red, send help")
                message = message_encode("BlockGreen",[blockid])
                emitter.send(message)
                blocks[blockid][2] = "green"
                blocks[blockid][3] = "ignore"
                return False
            else:
                print("This block is green and I am green, LETS GO")
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

    #Hardcoded to pick up the block
    openDoor()
    #start = time.time()
    
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
    print("The block has been sorted")
    return True

def drive_straight(leftSpeed,rightSpeed,t):
    #Function that makes the robot drive for a fixed period of time
    start = robot.getTime()
    while robot.getTime()-start <t:
        setSpeed(leftSpeed,rightSpeed)  
        robot.step(1)
       
"""This function was not utilised as well, but rather shows the expected flow of
the collision avoidance algorithm"""
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
    
if robot_colour == "red":
    #Pauses the red robot so it doesn't conflict with green's scanning
    passive_wait(8)

while robot.step(TIME_STEP) != -1:
    endThisSuffering()
       
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]

    angle = convert_compass_angle(compass.getValues())
 

    #Initial scan, robot does a 360 while finding blocks
    if robot_status == "initial scan":
        leftSpeed = -0.4
        rightSpeed = 0.4
        #Scans 360 degrees to find all boxes
        walldistLong =  dist_to_wall(angle,coord2d,"long")
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        #print("Predicted: " + str(walldistLong))
        #print("Actual :" + str(sensordistLong))
        sort_all_messages()
        if robot_colour == "green" or robot.getTime()-start>=2:
            scan(sensordistLong,walldistLong)
        
        
        if robot_colour == "green":
            if angle > 3*np.pi/2 and angle < 3*np.pi/2 + 0.1 and robot.getTime() - start >= 2:
                robot_status = "logic"
                leftSpeed = 0
                rightSpeed = 0
                setSpeed(leftSpeed,rightSpeed)
                passive_wait(6)
        else:
            if angle > 3*np.pi/2 and angle < 3*np.pi/2 + 0.1 and robot.getTime() - start >= 13:
                robot_status = "logic"
    """else:
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        sensordistShort = sensor_to_dist(ds_left.getValue(),sensorX,"short")
        if sensordistLong > 0.8 + sensorX:
            sensordistLong = 0.8 + sensorX
        if anomalies(sensordistLong,sensordistShort):
            scan(sensordistLong,sensordistShort)"""

            
    #Logic selects the closest potential block and directs the robot towards it
    if robot_status == "logic":
        shortest_distance = 10
        robot_status = "end"
        sort_all_messages()
        #print(blocks)
        if robot_colour == "red":
            passive_wait(0.2)
        for id in range(len(blocks)):
            
            if (blocks[id][3] == "Unsorted" and blocks[id][2] == "Unknown") or (blocks[id][2] == robot_colour and (blocks[id][3] == "Unsorted" or blocks[id][3] == "Bot2Chope")):
                distance = dist(coord2d[0],coord2d[1],blocks[id][0],blocks[id][1])
                #print([coord2d[0],coord2d[1],blocks[id][0],blocks[id][1]])
                #print(distance)
                if distance < shortest_distance:
                    shortest_distance = distance
                    
                    blockid = id

                
        if shortest_distance == 10 and (blocksCollected == 4 or len(scanWaypoints) == 0):
            robot_status = "end" 
        
        elif shortest_distance == 10 and blocksCollected != 4:
            #Haven't discovered all the blocks, going to alternative location to scan for more blocks
            destination = scanWaypoints[0]
            scanWaypoints.pop(0)
            move_to_coordinate(destination, 0.01)
            robot_status = 'initial scan'
            
        else:       
            destination = [blocks[blockid][0],blocks[blockid][1],"block"]
            message = message_encode("MyBlock",[blockid])
            emitter.send(message)
            print("Going to block: " + str(blockid))
            robot_status = "navigating" 
                
        

            
        #Either search for more or die
 
    
    if robot_status == "navigating":
        print("Navigating")
        if short_scan():
            blocksCollected += 1
        else:
            pass 
        robot_status = "logic"
      
    
    """if robot_status == "checking": 
    #Gets close to block and then checks the colour
    #Does not use previously scanned coordinates
        check block()
        
    if robot_status = "Idle":
    #selection logic to figure out what to do - also chooses block
    
    """
 
    """ls_red_value = ls_red.getValue()
    ls_green_value = ls_green.getValue()
    
    if ls_red_value > 0:
        print("red")
        
    if ls_green_value > 0:
        print("green")  """  
        
    #Ends the program   
    if robot_status == "end":
        move_to_coordinate(home,0.01)
        leftSpeed = 0
        rightSpeed = 0
        print("PROGRAM TERMINATED")
        sys.exit()
    

    
    sort_all_messages()
    

    setSpeed(leftSpeed,rightSpeed)
    """
    #If too much time elapsed just end
    """
    
    #print(angle)      
    #print("Wall dist: " + str(walldist))
    #print("Sensor dist: " + str(sensordist))
