from controller import Robot, Motor, Keyboard




 

TIME_STEP = 64

MAX_SPEED = 3.14

 

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

 
def moveToPosition(position):
    #moves wheels to position (in rads)
    left_wheel.setPosition(position)
    right_wheel.setPosition(position)

def move(speed):
    
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(speed*MAX_SPEED)
    right_wheel.setVelocity(speed*MAX_SPEED)

 

def turn(speed):
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(-speed*MAX_SPEED)
    right_wheel.setVelocity(speed*MAX_SPEED)

    

def openDoor():
    left_door.setPosition(1.57)
    right_door.setPosition(1.57)

    

def closeDoor():
    left_door.setPosition(0)
    right_door.setPosition(0)
    

turn(0.5)
closeDoor()

def passive_wait(time):
    start_time = robot.getTime()
    while start_time + time > robot.getTime():
        print(start_time + time)
        print(robot.getTime())
        robot.step(1)

while robot.step(timestep) != -1:
    #read sensors
    #ds_right_value = ds_right.getValue()
    #ds_left_value = ds_left.getValue()
    #if ds_left_value > 0.5:
    #    move(-0.5)
    #elif ds_left_value < 0.45:
    #    move(0.5)
    #ls_red_value = ls_red.getValue()
    #ls_green_value = ls_green.getValue()
    #key=keyboard.getKey()
    moveToPosition(1)
    passive_wait(200)
        
    

    
    #print sensor values
    #print("Right = ", ds_right_value,"Left = ", ds_left_value)
    #print("Red = ", ls_red_value, "Green = ", ls_green_value)
    pass