from controller import Robot,GPS,Compass,Receiver,Emitter
import numpy as np
import struct
import time


"""
Setup
"""
start = time.time()

blocks = []#format: [coord1,coord2,colour,sorted?]
other_robot_coords = [0,0.4]
current_block = None
block_hitbox_radius = 0.1 #0.056
robot_hitbox_radius = None
robot_status = "initial scan"
destination = [0.5,0.5]
MAX_SPEED = 3.14
sensorX = 0.115
sensorZ = 0
navigation_status = 0
WHEEL_RADIUS = 5.08
ROBOT_WIDTH = 18 + 2*WHEEL_RADIUS
TURN_RADIUS = (ROBOT_WIDTH-WHEEL_RADIUS+1.11)/2

TIME_STEP = 64
robot = Robot()

    
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
receiver.enable(100)


timestep = int(robot.getBasicTimeStep())



def turnRadian(radians):

    position = (radians*TURN_RADIUS)/(WHEEL_RADIUS)
    print(position)
    left_wheel.setPosition(-position)
    right_wheel.setPosition(position)
    left_wheel.setVelocity(MAX_SPEED)
    right_wheel.setVelocity(MAX_SPEED)

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

def convert_compass_angle(compass_values:list)->float:
    #Returns angle using polar angle system, horizontal axis = 0 rads
    """rad = np.pi/2 - np.arctan2(compass_values[0],compass_values[2])
    if rad <=0:
        rad += 2*np.pi
    return rad"""
    
    rad = -np.arctan2(compass_values[0],compass_values[2])
    if rad <=0:
        rad += 2*np.pi
    return rad

def dist_to_wall(angle:float,position:list,sensor_type)->float:
    #Finds what the distance sensor should be reading
    
    if sensor_type == "short":
        cap = 0.8
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

def get_object_position(position:list,angle:float,sensordist:float)->list:
    #Determines the coordinates of the box
    x = position[0]
    z = position[1]
    return [x+(sensordist+0.025) * np.sin(angle),z+(sensordist+0.025) * np.cos(angle)]#Adds to centroid, may need to be tuned     
        
def sensor_to_dist(sensor_value:float,bot_length:float,sensor_type)->float:
    #Uses sensitivity curve to calculate actual distance, factors in position of sensor
    #return (1000-sensor_value)/666.7 + bot_length
    
    #Linear interpolation bewteen 2 points
    #print("Value used: " +str(sensor_value))
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
    
        
    
    
def what_is_it(position:list,other_robot_position:list)->str:
    #Determines if the box is interesting
    x = position[0]
    z = position[1]  
    for block in blocks:
        if (hitboxcollision(x,z,block[0],block[1],block_hitbox_radius) == True) and (block[3] != "Sorted"):
            #print("This is a dupe, ignore it")
            return "ExistingBox"

    #if hitboxcollision(x,z,other_robot_coords[0],other_robot_coords[1],robot_hitbox_radius) == True:  
    if (x <= 0.2 and x >= -0.2 and ((z <=0.6 and z >= 0.2) or (z >=-0.6 and z <= -0.2))):
        print("This box has already been sorted")
        return "SortedBox"
    else:
        #print("X: " + str(x) + "Z: " + str(z))
        return "NewBox"
        
        
def hitboxcollision(x1:float,z1:float,x2:float,z2:float,r2:float)->bool:
    if (x2-x1)**2+(x2-x1)**2 <= r2**2:
        return True
    else:
        return False
        
def dist(x1:float,z1:float,x2:float,z2:float)->bool:
    return (x2-x1)**2+(x2-x1)**2

    
def passive_wait(time):
    start_time = robot.getTime()
    while start_time + time > robot.getTime():
        robot.step(1)
        
        
def message_encode(message_type:str,content:list)->bytes:
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
    return message

    
def message_decode(message:bytes)->(str,list):
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
            for item in blocks:
                if  item[3] == "Bot2Chope":
                    item[3] == "Unsorted"
            blocks[data[0]][3] = "Bot2Chope"
            #Also unchope other blocks
        receiver.nextPacket()
        #print(blocks)

def scan(sensordist,walldist):
    if sensordist < 1.55 and sensordist<walldist * 0.9:#Need to tune this value so no false positives
        #print("Object found at " + str(angle) + " Sensor dist: "+str(sensordist)+ " Wall dist: " + str(walldist))
        object_position = get_object_position(coord2d,angle,sensordist)
        status = what_is_it(object_position,1)
        if status == "NewBox":
            blocks.append([object_position[0],object_position[1],"Unknown","Unsorted"])
            message = message_encode("NewBlock",[object_position[0],object_position[1]])
            emitter.send(message)
 
        else:
            pass
            
def anomalies(sensorLong,sensorShort): 
    if abs(sensorLong - sensorShort) > 0.1:
        #print("Long: " + str(sensorLong))
        #print("Short: " + str(sensorShort))
        return True

while robot.step(TIME_STEP) != -1:
    
       
    coord3d = gps.getValues()
    coord2d = [coord3d[0],coord3d[2]]

    angle = convert_compass_angle(compass.getValues())
 
    
    
    if robot_status == "initial scan":
        leftSpeed = -0.4
        rightSpeed = 0.4
        #Scans 360 degrees to find all boxes
        walldistLong =  dist_to_wall(angle,coord2d,"long")
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        #print("Predicted: " + str(walldistLong))
        #print("Actual :" + str(sensordistLong))
        scan(sensordistLong,walldistLong)
        if angle > 3*np.pi/2 and angle < 3*np.pi/2 + 0.1 and time.time() - start >= 2:
            robot_status = "logic"
    else:
        sensordistLong = sensor_to_dist(ds_right.getValue(),sensorX,"long")
        sensordistShort = sensor_to_dist(ds_left.getValue(),sensorX,"short")
        if sensordistLong > 0.8 + sensorX:
            sensordistLong = 0.8 + sensorX
        if anomalies(sensordistLong,sensordistShort):
            scan(sensordistLong,sensordistShort)

            
            
    if robot_status == "logic":
        destination = [blocks[0][0],blocks[0][1],"block"]
        print(destination)
        robot_status = "navigating"
        #for block in blocks:
            
        
    
    if robot_status == "navigating":
        pass
        #This part is all trash
        
     """   
        #Do this if it is further than 60cm away
        if navigation_status == 0:
            theta_destination = -np.pi/2-np.arctan2(coord2d[1]-destination[1],coord2d[0]-destination[0])
            if theta_destination <=0:
                theta_destination += 2*np.pi
    
            test_angle = angle
            #print(theta_destination)
            #print(test_angle)
            
            if test_angle < theta_destination:
                test_angle+=np.pi*2
            if abs(theta_destination - angle) < 0.05:
                leftSpeed = -0.5
                rightSpeed = -0.5
                if hitboxcollision(coord2d[0],coord2d[1],destination[0],destination[1],0.18675):
                    navigation_status = 1
            
            elif  (test_angle - theta_destination) > np.pi:
                #print("Turning Left")
                leftSpeed = -1.0
                rightSpeed = 1.0
            elif (test_angle - theta_destination) <= np.pi:
                #print("Turning right")
                leftSpeed = 1.0
                rightSpeed = -1.0
            
            
        
        #Rotate until it sees block colour
        if navigation_status == 1:
        
            leftSpeed = -0.1
            rightSpeed = 0.1
            ls_red_value = ls_red.getValue()
            ls_green_value = ls_green.getValue()
            if ls_red_value > 0:
                robot_status = "end"
        
            if ls_green_value > 0:
                print("Green found")
                
                setSpeed(0.3,0.3)
                print("Reversing")
                openDoor()  
                passive_wait(1)
                setSpeed(0,0)
                turnRadian(np.pi)
                print("Turning")
                passive_wait(5)
                
                
                print("Turning")
                turnRadian(np.pi)
                passive_wait(5)
                print("Collecting")
                moveToPosition(0.3)
                passive_wait(5)
                closeDoor()
                break
                

                
        #Pick up the block      
        if navigation_status == 2: 
            
            leftSpeed = -0.3
            rightSpeed = 0.3

            
            test_angle = angle
            #print(theta_destination)
            #print(test_angle)
            
            if test_angle < theta_destination:
                test_angle+=np.pi*2
            if abs(theta_destination - angle) < 0.05:
                leftSpeed = 0.5
                rightSpeed = 0.5
                
                
                

                
            #print("close approach")
        

        

        
        #Do this if it is less than 40cm away

    
        
    """    
    
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
        
        
    if robot_status == "end":
        leftSpeed = 0
        rightSpeed = 0
    

    
    sort_all_messages()
    

    setSpeed(leftSpeed,rightSpeed)
    
    #print(angle)      
    #print("Wall dist: " + str(walldist))
    #print("Sensor dist: " + str(sensordist))