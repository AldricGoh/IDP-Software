from constants import *
from setup import *


class Status:
    def __init__(self):
        self.idle = True
        self.scanning = False
        self.moving_to_box = False
        self.moving_to_base = False
        self.aligning = False
        self.fine_searching = False
        self.collecting = False
        self.turning_speed = 0
        self.revolutions = 0
        self.previous_heading = 0
        self.linear_speed = 0
        
    def __str__(self):
        if(self.idle):return "idle"
        elif(self.scanning):return "scanning"
        elif(self.moving_to_box):return "moving to box"
        elif(self.moving_to_base):return "moving to base"
        elif(self.fine_searching):return "fine searching"
        elif(self.collecting):return "collecting"
        
    def reset(self):
        self.idle = False
        self.scanning = False
        self.scan = None
        self.moving_to_box = False
        self.moving_to_base = False
        self.fine_searching = False
        self.collecting = False
        self.turn(0)
        self.move(0)
        
    def move(self, speed):
        left_wheel.setPosition(float('inf'))
        right_wheel.setPosition(float('inf'))
        left_wheel.setVelocity(speed*MAX_SPEED)
        right_wheel.setVelocity(speed*MAX_SPEED)
        self.linear_speed = speed
    
    def turn(self, speed):
        left_wheel.setPosition(float('inf'))
        right_wheel.setPosition(float('inf'))
        left_wheel.setVelocity(-speed*MAX_SPEED)
        right_wheel.setVelocity(speed*MAX_SPEED)  
        self.turning_speed = speed  
        
    def start_idle(self):
        self.reset()
        self.idle = True
        
    def start_moving(self):
        self.reset()
        self.moving_to_box = True
        
    def start_scan(self, initial_heading):
        self.reset()
        self.scanning = True
        self.scan = Scan(initial_heading)
        self.turn(SCAN_SPEED)
        
class Scan:
    def __init__(self, initial_heading):
        self.initial_heading = initial_heading
        self.dists_bottom = []
        self.dists_top = []
        self.angles = []
        self.positions = []