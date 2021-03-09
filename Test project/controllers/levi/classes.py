import numpy as np

from constants import *
from setup import *


class Status:
    def __init__(self):
        self.start_idle()
        self.revolutions = 0
        self.previous_heading = 0
        
    def __str__(self):
        if self.idle: return "idle"
        elif self.scanning: return "scanning"
        elif self.moving_to_box: return "moving to box"
        elif self.moving_to_base: return "moving to base"
        elif self.fine_searching: return "fine searching"
        elif self.collecting: return "collecting"
        
    def reset(self):
        self.idle = False
        self.scanning = False
        self.scan = None
        self.aligning = False
        self.align = None
        self.moving_to_box = False
        self.move_to_box = None
        self.aligning = False
        self.moving_to_base = False
        self.fine_searching = False
        self.collecting = False
        self.got_box = False
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
        
    def start_scan(self, initial_heading):
        self.reset()
        self.scanning = True
        self.scan = Scan(initial_heading)
        self.turn(SCAN_SPEED)
        print("Starting scan")
        
    def start_align(self, initial_heading, initial_position, target):
        self.reset()
        self.aligning = True
        # (Initializing an Align object automaticaly calculates the closest equivalent target heading)
        self.align = Align(initial_heading, initial_position, target)
        self.turn(np.sign(self.align.target_heading-self.align.initial_heading)*ALIGN_SPEED)
        print("Aligning to {}, at heading {:.2f}". format(target, self.align.target_heading%(2*np.pi)/(2*np.pi)*360))        
        
    def start_move_to_box(self, box):
        self.reset()
        self.moving_to_box = True 
        self.   
    
    
        
        
class Scan:
    def __init__(self, initial_heading):
        self.initial_heading = initial_heading
        self.dists_bottom = []
        self.dists_top = []
        self.angles = []
        self.positions = []
    
    def get_object_position(self, dist, angle, robot_position):
        """Calculate position estimate for object from distance sensor reading and robot orientation and position
           Rounds to closest mm"""
        dist = dist + DISTANCE_SENSOR_OFFSET
        position = [round(robot_position[0] + dist*np.sin(angle), 2),
                    round(robot_position[1] + dist*np.cos(angle), 2)]
        return position
         
    def evaluate_scan(self):
        """Evaluates the data gathered in the scan and returns a list of box positions"""
        boxes = []
        
        # Get all box position estimates from the scan (rough list with many duplicates)
        for i in range(len(self.angles)):
            if self.dists_bottom[i] < 0.8 and np.abs(self.dists_bottom[i]-self.dists_top[i]) > 0.04:
                boxes.append(self.get_object_position(self.dists_bottom[i], self.angles[i], self.positions[i]))
        
        # Convert to BoxesList object
        boxes = BoxList(boxes)
        
        # Filter boxes list (refined list with hopefully one entry for each physical box)
        boxes.filter()
        
        return boxes
        
class Align:
    def __init__(self, initial_heading, initial_position, target):
        self.initial_heading = initial_heading
        
        # Calculate the ideal target_heading taking into account possible turning directions
        # Calculate target_heading (0-2pi) and add revolutions to catch it up to the true_heading
        relative_position = [target[0]-initial_position[0],
                             target[1]-initial_position[1]]
        target_heading = np.arctan2(relative_position[0], relative_position[1]) 
        target_heading += (2*np.pi) * (int(self.initial_heading/(2*np.pi)))
        
        # Determine closest equivalent heading to the initial_heading
        ideal_target_heading = target_heading
        for i in range(-1,2):
            if np.abs(target_heading+i*(2*np.pi) - initial_heading) < np.abs(ideal_target_heading - initial_heading):
                ideal_target_heading = target_heading+i*(2*np.pi)
                                   
        self.target_heading = ideal_target_heading
        
class MoveToBox:
    def __init__(self, target):
        self.target = target

class BoxList:
    def __init__(self, boxes):
        self.boxes = boxes
        
    def __str__(self):
        return repr(self.boxes)
        
    def __len__(self):
        return len(self.boxes)
        
    def __getitem__(self, i):
        return self.boxes[i]
        
    def __iter__(self):
        return iter(self.boxes)
        
    def distance(self, box1, box2):
        """Calculate the distance between two boxes"""
        return np.sqrt((box1[0]-box2[0])**2 + (box1[1] - box2[1])**2)
        
    def average_group(self, group):
        """Get average position for a group of closeby box position estimates"""
        group = np.array(group)
        size = len(group)
        average = [round(sum(group[:,0])/size, 2),
                   round(sum(group[:,1])/size, 2)]       
        return average
        
    def filter(self):
        """Compile a filtered list by getting rid of duplicates and avaraging closeby items that likely correspond to the same box"""
        boxes = self.boxes
        
        # Get rid of boxes that show up on less than 3 readings (these are likely to be errors)
        # We found that even on a 100% SCAN_SPEED boxes tend to show up on 4 consecutive readings
        for box in self.boxes:
            if boxes.count(box) < 3:
                   boxes.remove(item)
        
        # Merge duplicates of the same box (this approach should be very quick compared to nested for loops)
        # Convert to tuple of tuples because we need a hashable type to convert to a set
        boxes = tuple([tuple(box) for box in boxes])
        # Convert to a set to get rid of duplicates (sets dont allow duplicates)
        boxes = set(boxes)
        # Convert back to list of lists
        boxes = [list(box) for box in boxes]
        
        # Merge boxes that are less than 7cm away (this is the length of the diagonal of a box)
        groups = []
        for box1 in boxes:
            group = [box1]
            for box2 in boxes:
                if box1 != box2 and self.distance(box1, box2) < 0.07:
                    group.append(box2)
                    boxes.remove(box2)
            groups.append(group)
        
        boxes = []
        for group in groups:
            boxes.append(self.average_group(group))
              
        self.boxes = boxes
        
    def closest_to_position(self, position):
        """Get closest box to a position"""
        closest = self.boxes[0]
        for box in self.boxes:
            if self.distance(box, position) < self.distance(closest, position):
                closest = box
                
        return closest
  
            
          
        
        