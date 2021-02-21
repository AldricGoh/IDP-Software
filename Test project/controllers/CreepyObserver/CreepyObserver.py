"""CreepyObserver controller."""
from controller import Receiver,Emitter
from controller import Robot
import struct

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Emitter Receiver
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
emitter.setChannel(2)
receiver.setChannel(1)
receiver.enable(100)

while robot.step(timestep) != -1:
    
    if receiver.getQueueLength() > 0:
        message = receiver.getData()
        data=struct.unpack("c",message)
        receiver.nextPacket()
        print(data)

# Enter here exit cleanup code.
