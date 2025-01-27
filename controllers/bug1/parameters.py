# Configuration Parameters
MAX_SPEED = 6.28            
WALL_THRESHOLD = 0.34       # Wall detection distance [m]
TURN_FACTOR = 0.25           # turn gain
TURN_ANGLE = 90             # Default turn angle [degrees]
MOVE_DISTANCE = 0.07        # Straight movement distance for circumvention [m]
MOVE_DISTANCE2 = 0.17       # Straight movement distance for circumvention - step 2[m]
KP_ANGLE = 0.2              # IMU turn control gain
WHEEL_RADIUS = 0.033        # Wheel radius [m]

# State machine states
STATE_NORMAL = 0
STATE_CIRCUMVENT = 1
STATE_RETURNING = 2

STATE_STOP = 4

Q_GOAL = [4.27, -2.27]  