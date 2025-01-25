from controller import Robot, DistanceSensor, InertialUnit, PositionSensor
import math

# Configuration Parameters
MAX_SPEED = 6.28            
WALL_THRESHOLD = 0.3        # Wall detection distance [m]
TURN_FACTOR = 0.25          # turn gain
TURN_ANGLE = 90             # Default turn angle [degrees]
MOVE_DISTANCE = 0.07        # Straight movement distance for circumvention [m]
MOVE_DISTANCE2 = 0.17       # Straight movement distance for circumvention - step 2[m]
KP_ANGLE = 0.2              # IMU turn control gain
WHEEL_RADIUS = 0.033        # Wheel radius [m]

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Configure motors
wheels = {
    'front_left': robot.getDevice('front left wheel'),
    'front_right': robot.getDevice('front right wheel'),
    'back_left': robot.getDevice('back left wheel'),
    'back_right': robot.getDevice('back right wheel')
}
for wheel in wheels.values():
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)

# Configure sensors
sensors = {
    'left': robot.getDevice('so0'),
    'center': robot.getDevice('so4'),
    'right': robot.getDevice('so7'),
    'back_left': robot.getDevice('so15'),
    'back_right': robot.getDevice('so8'),
    'imu': robot.getDevice('inertial unit'),
    'enc_left': robot.getDevice('front left wheel sensor'),
    'enc_right': robot.getDevice('front right wheel sensor')
}
for s in sensors.values():
    if isinstance(s, (DistanceSensor, InertialUnit, PositionSensor)):
        s.enable(timestep)

# State machine states
STATE_NORMAL = 0
STATE_CIRCUMVENT = 1

current_state = STATE_NORMAL
current_wall_side = None
original_wall_side = None  # For L-shape detection
maneuver_step = 0
initial_encoder = 0

def sensor_to_distance(sensor):
    return (1024 - sensor.getValue()) / 204.8

def set_speeds(left, right):
    wheels['front_left'].setVelocity(left)
    wheels['back_left'].setVelocity(left)
    wheels['front_right'].setVelocity(right)
    wheels['back_right'].setVelocity(right)

def get_yaw():
    return math.degrees(sensors['imu'].getRollPitchYaw()[2]) % 360

def rotate(target_angle):
    """Precision turn using IMU"""
    while robot.step(timestep) != -1:
        current = get_yaw()
        error = (target_angle - current + 180) % 360 - 180
        if abs(error) < 1.0:
            break
        speed = KP_ANGLE * error
        speed = max(min(speed, MAX_SPEED), -MAX_SPEED)
        set_speeds(-speed, speed)

def get_travel_distance():
    """Get distance traveled in meters using encoder radians"""
    return (sensors['enc_left'].getValue() + 
            sensors['enc_right'].getValue()) / 2 * WHEEL_RADIUS

def start_circumvention(direction):
    global current_state, maneuver_step, initial_encoder, original_wall_side
    current_state = STATE_CIRCUMVENT
    maneuver_step = 0
    initial_encoder = get_travel_distance()
    original_wall_side = current_wall_side  
    print(f"Starting {direction} circumvention")

while robot.step(timestep) != -1:
    # Read sensors
    front = sensor_to_distance(sensors['center'])
    left = sensor_to_distance(sensors['left'])
    right = sensor_to_distance(sensors['right'])
    back_left = sensor_to_distance(sensors['back_left'])
    back_right = sensor_to_distance(sensors['back_right'])

    if current_state == STATE_NORMAL:
        if front < WALL_THRESHOLD:
            if left > right:
                print("Front wall - turning left")
                rotate((get_yaw() + TURN_ANGLE) % 360)
                current_wall_side = 'right'
            else:
                print("Front wall - turning right")
                rotate((get_yaw() - TURN_ANGLE) % 360)
                current_wall_side = 'left'
            continue

        if current_wall_side == 'left':
            if left < WALL_THRESHOLD:
                error = (WALL_THRESHOLD - left) * TURN_FACTOR
                set_speeds(MAX_SPEED - error, MAX_SPEED + error)
            else:
                start_circumvention('left')
        elif current_wall_side == 'right':
            if right < WALL_THRESHOLD:
                error = (WALL_THRESHOLD - right) * TURN_FACTOR
                set_speeds(MAX_SPEED + error, MAX_SPEED - error)
            else:
                start_circumvention('right')
        else:
            set_speeds(MAX_SPEED, MAX_SPEED)

    elif current_state == STATE_CIRCUMVENT:
        traveled = get_travel_distance() - initial_encoder
        
        if maneuver_step == 0:  # First straight
            if traveled < MOVE_DISTANCE:
                set_speeds(MAX_SPEED, MAX_SPEED)
            else:
                rotate(get_yaw() + (-TURN_ANGLE if original_wall_side == 'right' else TURN_ANGLE))
                maneuver_step = 1
                initial_encoder = get_travel_distance()
                
        elif maneuver_step == 1:  # Second straight 
            if traveled < MOVE_DISTANCE2:
                set_speeds(MAX_SPEED, MAX_SPEED)
            else:
                # Check for L-shape
                new_wall_side = original_wall_side 
                distance = sensor_to_distance(sensors[new_wall_side])
                
                if distance < WALL_THRESHOLD:
                    # L-shape detected - follow new wall
                    current_wall_side = new_wall_side
                    current_state = STATE_NORMAL
                    print(f"L-shape detected, now following {new_wall_side} wall")
                else:
                    # Complete full circumvention
                    rotate(get_yaw() + (-TURN_ANGLE if original_wall_side == 'right' else TURN_ANGLE))
                    maneuver_step = 2
                    initial_encoder = get_travel_distance()
                    print("Proceeding with full circumvention")

        elif maneuver_step == 2:  # Final straight
            if traveled < MOVE_DISTANCE2:
                set_speeds(MAX_SPEED, MAX_SPEED)
            else:
                current_state = STATE_NORMAL
                current_wall_side = original_wall_side
                print("Circumvention complete")