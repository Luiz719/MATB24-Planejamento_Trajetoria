from controller import Robot, DistanceSensor, InertialUnit, PositionSensor
from parameters import *
import math

class Circumvention():
    def __init__(self, robot):
        self.robot = robot
        self.current_state = STATE_NORMAL
        self.current_wall_side = None
        self.maneuver_step = 0
        self.initial_encoder = 0
    
    def sensor_to_distance(self, sensor):
        return (1024 - sensor.getValue()) / 204.8

    def set_speeds(self, left, right):

        self.robot.wheels['front_left'].setVelocity(left)
        self.robot.wheels['back_left'].setVelocity(left)
        self.robot.wheels['front_right'].setVelocity(right)
        self.robot.wheels['back_right'].setVelocity(right)

    def get_yaw(self):
        return math.degrees(self.robot.sensors['imu'].getRollPitchYaw()[2]) % 360

    def rotate(self, target_angle, o=False):
        """Precision turn using IMU"""
        while self.robot.step() != -1:
            current = self.get_yaw()
            if o:
                print("current=",current)
            error = (target_angle - current + 180) % 360 - 180
            if abs(error) < 1.0:
                break
            speed = KP_ANGLE * error
            speed = max(min(speed, MAX_SPEED), -MAX_SPEED)
            self.set_speeds(-speed, speed)

    def get_travel_distance(self):
        """Get distance traveled in meters using encoder radians"""
        return (self.robot.sensors['enc_left'].getValue() + 
                self.robot.sensors['enc_right'].getValue()) / 2 * WHEEL_RADIUS

    def start_circumvention(self, direction):
        self.current_state = STATE_CIRCUMVENT
        self.maneuver_step = 0
        self.initial_encoder = self.get_travel_distance()
        self.current_wall_side = self.current_wall_side  
        print(f"Starting {direction} circumvention")

    def move(self):
        # Read sensors
        front = self.sensor_to_distance(self.robot.sensors['center'])
        left = self.sensor_to_distance(self.robot.sensors['left'])
        right = self.sensor_to_distance(self.robot.sensors['right'])
        if front < WALL_THRESHOLD:
            obstacle_entry = self.robot.position
            if left > right:
                print("Front wall - turning left")
                self.rotate((self.get_yaw() + TURN_ANGLE) % 360)
                self.current_wall_side = 'right'
            else:
                print("Front wall - turning right")
                self.rotate((self.get_yaw() - TURN_ANGLE) % 360)
                self.current_wall_side = 'left'
            return self.current_state, obstacle_entry

        if self.current_wall_side == 'left':
            if left < WALL_THRESHOLD:
                error = (WALL_THRESHOLD - left) * TURN_FACTOR
                self.set_speeds(MAX_SPEED - error, MAX_SPEED + error)
            else:
                self.start_circumvention('left')
        elif self.current_wall_side == 'right':
            if right < WALL_THRESHOLD:
                error = (WALL_THRESHOLD - right) * TURN_FACTOR
                self.set_speeds(MAX_SPEED + error, MAX_SPEED - error)
            else:
                self.start_circumvention('right')
        else:
            if left > right:
                error = (WALL_THRESHOLD - right) * 3*TURN_FACTOR
                self.set_speeds(MAX_SPEED - error, MAX_SPEED + error)
            else: 
                error = (WALL_THRESHOLD - left) * 3*TURN_FACTOR
                self.set_speeds(MAX_SPEED + error, MAX_SPEED - error)

        return self.current_state, None
    
    def returning(self):
        if self.current_state == STATE_CIRCUMVENT:
            self.circumvent_obstacle()
        else:
            self.move()

    def circumvent_obstacle(self):
    
        if self.current_state == STATE_CIRCUMVENT:
            traveled = self.get_travel_distance() - self.initial_encoder
            
            if self.maneuver_step == 0:  # First straight
                if traveled < MOVE_DISTANCE:
                    self.set_speeds(MAX_SPEED, MAX_SPEED)
                else:
                    self.rotate(self.get_yaw() + (-TURN_ANGLE if self.current_wall_side == 'right' else TURN_ANGLE))
                    self.maneuver_step = 1
                    self.initial_encoder = self.get_travel_distance()
                    
            elif self.maneuver_step == 1:  # Second straight 
                if traveled < MOVE_DISTANCE2:
                    self.set_speeds(MAX_SPEED, MAX_SPEED)
                else:
                    # Check for L-shape
                    new_wall_side = self.current_wall_side 
                    distance = self.sensor_to_distance(self.robot.sensors[new_wall_side])
                    
                    if distance < WALL_THRESHOLD:
                        # L-shape detected - follow new wall
                        self.current_wall_side = new_wall_side
                        self.current_state = STATE_NORMAL
                        print(f"L-shape detected, now following {new_wall_side} wall")
                    else:
                        # Complete full circumvention
                        self.rotate(self.get_yaw() + (-TURN_ANGLE if self.current_wall_side == 'right' else TURN_ANGLE))
                        self.maneuver_step = 2
                        self.initial_encoder = self.get_travel_distance()
                        print("Proceeding with full circumvention")

            elif self.maneuver_step == 2:  # Final straight
                if traveled < MOVE_DISTANCE2:
                    self.set_speeds(MAX_SPEED, MAX_SPEED)
                else:
                    self.current_state = STATE_NORMAL
                    self.current_wall_side = self.current_wall_side
                    print("Circumvention complete")
        
        return self.current_state