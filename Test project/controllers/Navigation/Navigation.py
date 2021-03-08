from controller import Robot, Motor, Keyboard, Compass, GPS
import numpy as np
import time
from functions import *
from setup import *

print('start')

timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
    # coord3d = gps.getValues()
    # coord2d = [coord3d[0],coord3d[2]]

    # moveToPosition(1)
    
    # passive_wait(2)
    # turnRadian(2)
    align(-0.01)
    # passive_wait(2)
    
    # break

    pass
    
