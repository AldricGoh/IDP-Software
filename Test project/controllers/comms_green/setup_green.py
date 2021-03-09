from controller import Robot, Motor, Compass, GPS, Emitter, Receiver

from constants import *

ROBOT_COLOR = 'green'

# Devices
robot = Robot()
# --Motors
# left_wheel = robot.getDevice("left_wheel")
# right_wheel = robot.getDevice("right_wheel")
# left_door = robot.getDevice("left_door")
# right_door = robot.getDevice("right_door")
# --Sensors
# ds_top = robot.getDevice('ds_short')
# ds_bottom = robot.getDevice('ds_long')
# ls_red = robot.getDevice('ls_red')
# ls_green = robot.getDevice('ls_green')
# gps = robot.getDevice("gps")
# compass = robot.getDevice("compass")
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")

# Initial setup
# --Set maximum velocities
# left_wheel.setVelocity(MAX_SPEED)
# right_wheel.setVelocity(MAX_SPEED)
# --Enable sensors
# ds_top.enable(TIME_STEP)
# ds_bottom.enable(TIME_STEP)
# ls_red.enable(TIME_STEP) 
# ls_green.enable(TIME_STEP)
# gps.enable(TIME_STEP)
# compass.enable(TIME_STEP)
receiver.enable(TIME_STEP)

#Change robot channels
emitter.setChannel(2)
receiver.setChannel(1)