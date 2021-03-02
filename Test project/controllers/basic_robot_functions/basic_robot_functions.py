from controller import Robot, Motor

 

TIME_STEP = 64

MAX_SPEED = 3.14

 

robot = Robot()

 

left_wheel = robot.getDevice("left_wheel")

right_wheel = robot.getDevice("right_wheel")

left_door = robot.getDevice("left_door")

right_door = robot.getDevice("right_door")

 

timestep = int(robot.getBasicTimeStep())

 

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

    

def open_door():

    left_door.setPosition(1.57)

    right_door.setPosition(1.57)

    

def close_door():

    left_door.setPosition(0)

    right_door.setPosition(0)

    

    

close_door()

move(1)

 

while robot.step(timestep) != -1:

    pass