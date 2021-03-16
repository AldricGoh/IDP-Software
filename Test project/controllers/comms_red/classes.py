import numpy as np
import struct
from constants import *
from setup import *



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
        
    def merge(self, other_box_list):
        """Merge another BoxList object into the BoxList object and filter for duplicates and closeby ones"""
        self.boxes.append(other_box_list.boxes)
        self.filter()
        
    def remove(self, box):
        """Delete a box"""
        self.boxes.remove(box)
        
    def append(self, box):
        """Delete a box"""
        self.boxes.append(box)
        
    def closest_to_position(self, position):
        """Get closest box to a position"""
        closest = self.boxes[0]
        for box in self.boxes:
            if self.distance(box, position) < self.distance(closest, position):
                closest = box
                
        return closest
        

