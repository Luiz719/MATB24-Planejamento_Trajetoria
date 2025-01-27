from controller import Robot, Motor, GPS, Compass, DistanceSensor
from wall_circumvention import *
import sys
import math

class Pioneer3at:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Configure motors
        self.wheels = {
            'front_left': self.robot.getDevice('front left wheel'),
            'front_right': self.robot.getDevice('front right wheel'),
            'back_left': self.robot.getDevice('back left wheel'),
            'back_right': self.robot.getDevice('back right wheel')
        }
        for wheel in self.wheels.values():
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)

        # Configure sensors
        self.sensors = {
            'left': self.robot.getDevice('so0'),
            'center': self.robot.getDevice('so4'),
            'right': self.robot.getDevice('so7'),
            'back_left': self.robot.getDevice('so15'),
            'back_right': self.robot.getDevice('so8'),
            'imu': self.robot.getDevice('inertial unit'),
            'enc_left': self.robot.getDevice('front left wheel sensor'),
            'enc_right': self.robot.getDevice('front right wheel sensor')
        }
        for s in self.sensors.values():
            if isinstance(s, (DistanceSensor, InertialUnit, PositionSensor)):
                s.enable(self.timestep)

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
    
    @property
    def position(self):
        return self.gps.getValues()[:2]
    
    def set_speeds(self, left, right):
        left = max(min(left, 4), 0)
        right = max(min(right, 4), 0)
        self.wheels['front_left'].setVelocity(left)
        self.wheels['back_left'].setVelocity(left)
        self.wheels['front_right'].setVelocity(right)
        self.wheels['back_right'].setVelocity(right)
        
    def stop(self):
        self.set_speeds(0,0)
        sys.exit(0)
        
    def step(self):
        return self.robot.step(self.timestep)

class Bug1:
    def __init__(self, robot, goal_position):
        self.robot = robot
        self.goal = goal_position
        self.state = STATE_NORMAL
        
        self.goal_threshold = 0.2
        self.return_threshold = 0.1
        
        self.obstacles_pos = []
        self.obstacle_entry = None
        self.closest_point = None
        self.min_distance = float('inf')
        self.init_first_route_time = None
        self.init_sec_route_time = None
        self.first_route_duration = None
        self.sec_route_duration = None

        self.movement = Circumvention(self.robot)

    def check_return_finish(self):
        current_pos = self.robot.position
        if self.obstacle_entry is not None and self.distance_between(current_pos, self.obstacle_entry) < self.return_threshold:
            print("Completed boundary circumnavigation")
            self.sec_route_duration = self.robot.robot.getTime()-self.init_sec_route_time
            self.init_first_route_time = None
            self.state = STATE_RETURNING

    def boundary_following(self):
        """Follow obstacle boundary until loop closure"""
        current_pos = self.robot.position
        current_dist = self.distance_to_goal()
        
        # Update closest point
        if current_dist < self.min_distance:
            self.min_distance = current_dist
            self.closest_point = current_pos
            self.first_route_duration = self.robot.robot.getTime() - self.init_first_route_time
            self.init_sec_route_time = self.robot.robot.getTime()
            
        self.state = self.movement.circumvent_obstacle()
        current_pos = self.robot.position

        # Check loop closure
        self.check_return_finish()

    
    def move_to_closest_point(self):
        if self.first_route_duration > self.sec_route_duration:
            self.movement.rotate(self.movement.get_yaw() + (2*TURN_ANGLE if self.movement.current_wall_side == 'right' else -2*TURN_ANGLE) % 360)
            self.movement.current_wall_side = 'right' if  self.movement.current_wall_side == 'left' else 'left'
            self.first_route_duration = self.sec_route_duration
        
        self.movement.returning()
        if self.distance_between(self.robot.position, self.closest_point) < 0.56:
            print("Closest point reached")
            self.state = STATE_NORMAL
            self.closest_point = None
            self.obstacle_entry = None
            self.robot.stop()
 

    def distance_to_goal(self):
        return math.hypot(self.goal[0]-self.robot.position[0],
                         self.goal[1]-self.robot.position[1])
    
    def distance_between(self, p1, p2):
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    def is_obstacle_traversed(self, obstacle_pos):
        if not len(self.obstacles_pos) or obstacle_pos is None:
            return False
        for obst in self.obstacles_pos:
            if self.distance_between(obstacle_pos, obst):
                return True
        return False
    

    def at_goal(self):
        return self.distance_to_goal() < self.goal_threshold
    
    def run(self):
        while self.robot.step() != -1:
            if self.at_goal():
                print("Goal reached!")
                self.robot.stop()
                return
        
            if self.state == STATE_NORMAL:
                self.state, entry = self.movement.move()
                if entry is not None and self.obstacle_entry is None and not self.is_obstacle_traversed(entry):
                    self.obstacle_entry = entry
                    self.obstacles_pos.append(entry)
                else:
                    self.check_return_finish()

            elif self.state == STATE_CIRCUMVENT:
                if self.init_first_route_time is None:
                    self.init_first_route_time = self.robot.robot.getTime()
                self.boundary_following()

            elif self.state == STATE_RETURNING:
                self.move_to_closest_point()


if __name__ == "__main__":
    pioneer = Pioneer3at()
    navigator = Bug1(pioneer, Q_GOAL)
    
    navigator.run()