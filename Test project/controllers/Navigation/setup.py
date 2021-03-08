from controller import Robot, Motor, Keyboard, Compass, GPS
import numpy as np
import time

TIME_STEP = 16
MAX_SPEED = 3

#ROBOT CONSTANTS
WHEEL_RADIUS = 5.08
ROBOT_WIDTH = 18 + 2*WHEEL_RADIUS

# --DO NOT EDIT--
TURN_RADIUS = (ROBOT_WIDTH-WHEEL_RADIUS+1.11)/2
angles = []
distances = []
objects = []
robotStatus = 'START'
# --DO NOT EDIT--

robot = Robot()

#get robot motor devices
left_wheel = robot.getDevice("left_wheel")
right_wheel = robot.getDevice("right_wheel")
left_door = robot.getDevice("left_door")
right_door = robot.getDevice("right_door")
left_wheel.setVelocity(MAX_SPEED)
right_wheel.setVelocity(MAX_SPEED)
    
#get robot sensor devices
ds_bottom = robot.getDevice('ds_long')
ds_top= robot.getDevice('ds_short')
ls_red = robot.getDevice('ls_red')
ls_green = robot.getDevice('ls_green')
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")

#enable sensors
ds_bottom.enable(TIME_STEP)
ds_top.enable(TIME_STEP)
ls_red.enable(TIME_STEP) 
ls_green.enable(TIME_STEP)
gps.enable(TIME_STEP)
compass.enable(TIME_STEP)
#Emitter Receiver
