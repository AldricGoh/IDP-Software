# Constants
# --Fixed
TIME_STEP = 64
MAX_SPEED = 3.14

# --Determined
WHEEL_RADIUS = 5.08
ROBOT_WIDTH = 18 + 2*WHEEL_RADIUS
TURN_RADIUS = (ROBOT_WIDTH-WHEEL_RADIUS+1.11)/2
DISTANCE_SENSOR_OFFSET = 0.115
COLOR_SENSOR_OFFSET = 0.187
# --Adjustable
SCAN_SPEED = 1
ALIGN_SPEED = 1
FINE_ALIGN_SPEED = 0.1                  # last 0.1 radian of align sequence will happen at FINE_ALIGN_SPEED
MOVE_SPEED = 1