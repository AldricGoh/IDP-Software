import numpy as np

boxes = [[1,2], [4,5], [1,3], [1,2]]
print(boxes)
boxes = tuple([tuple(box) for box in boxes])
boxes = set(boxes)
boxes = [list(box) for box in boxes]
print(repr(boxes))
print(sum(np.array(boxes)[:,1]))
