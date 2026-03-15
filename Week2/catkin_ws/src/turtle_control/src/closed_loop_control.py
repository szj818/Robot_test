#!/usr/bin/env python3
"""
Closed-loop trajectory control node for turtlesim.
Implements a hexagon trajectory using PID controllers based on pose feedback.
"""
import rospy
import math
import csv
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class PIDController:
    """PID controller for closed-loop control"""

    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        """Update PID controller and return control output"""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        if self.output_limit:
            output = max(min(output, self.output_limit), -self.output_limit)
        return output

    def reset(self):
        """Reset internal state"""
        self.prev_error = 0
        self.integral = 0


class DataLogger:
    """Log control data to CSV file for analysis"""

    def __init__(self, filename):
        self.filename = filename
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'current_x', 'current_y', 'current_theta',
                           'target_x', 'target_y', 'target_angle',
                           'position_error', 'angle_error',
                           'linear_velocity', 'angular_velocity',
                           'vertex_index'])

    def log(self, data):
        """Log a data row"""
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(data)


class ClosedLoopControl:
    """Closed-loop control node for hexagon trajectory"""

    def __init__(self):
        # Initialize node
        rospy.init_node('closed_loop_control')

        # PID parameters (hardcoded as specified)
        # Position PID
        self.position_pid = PIDController(kp=1.0, ki=0.01, kd=0.1, output_limit=2.0)
        # Angle PID
        self.angle_pid = PIDController(kp=2.0, ki=0.05, kd=0.2, output_limit=1.5)

        # Target threshold
        self.position_threshold = 0.05  # meters

        # Trajectory parameters
        self.side_length = 2.0
        self.radius = self.side_length  # Hexagon circumradius
        self.center = (5.5, 5.5)  # turtlesim window center

        # Calculate hexagon vertices
        self.vertices = self.calculate_hexagon_vertices()

        # State variables
        self.current_pose = Pose()
        self.vertex_index = 0
        self.current_target = self.vertices[0]
        self.goal_reached = False

        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Data logger
        data_file = '/home/sunzj/Robot/Robot_test/Week2/catkin_ws/closed_loop_data.csv'
        self.logger = DataLogger(data_file)

        # Control loop timing
        self.control_rate = rospy.Rate(50)  # 50 Hz
        self.last_time = time.time()

        rospy.loginfo("Closed-loop control node initialized")
        rospy.loginfo("Hexagon vertices: %s", self.vertices)
        rospy.loginfo("Data will be logged to: %s", data_file)

    def calculate_hexagon_vertices(self):
        """Calculate hexagon vertex coordinates"""
        vertices = []
        for i in range(6):
            angle = i * math.pi / 3  # 60 degree intervals
            x = self.center[0] + self.radius * math.cos(angle)
            y = self.center[1] + self.radius * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def pose_callback(self, msg):
        """Handle incoming pose messages"""
        self.current_pose = msg

    def calculate_position_error(self):
        """Calculate distance to current target vertex"""
        dx = self.current_target[0] - self.current_pose.x
        dy = self.current_target[1] - self.current_pose.y
        return math.sqrt(dx**2 + dy**2)

    def calculate_target_angle(self):
        """Calculate target heading angle towards current vertex"""
        dx = self.current_target[0] - self.current_pose.x
        dy = self.current_target[1] - self.current_pose.y
        return math.atan2(dy, dx)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def run(self):
        """Main control loop"""
        rospy.loginfo("Starting closed-loop hexagon trajectory...")

        while not rospy.is_shutdown() and not self.goal_reached:
            # Calculate time delta
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            # Calculate errors
            position_error = self.calculate_position_error()
            target_angle = self.calculate_target_angle()
            angle_error = self.normalize_angle(target_angle - self.current_pose.theta)

            # PID control outputs
            linear_velocity = self.position_pid.update(position_error, dt)
            angular_velocity = self.angle_pid.update(angle_error, dt)

            # Create and publish twist message
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist)

            # Log data
            log_data = [
                rospy.Time.now().to_sec(),
                self.current_pose.x, self.current_pose.y, self.current_pose.theta,
                self.current_target[0], self.current_target[1], target_angle,
                position_error, angle_error,
                linear_velocity, angular_velocity,
                self.vertex_index
            ]
            self.logger.log(log_data)

            # Check if target reached
            if position_error < self.position_threshold:
                rospy.loginfo("Reached vertex %d at (%.2f, %.2f)",
                            self.vertex_index + 1, self.current_target[0], self.current_target[1])

                # Move to next vertex
                self.vertex_index = (self.vertex_index + 1) % 6
                self.current_target = self.vertices[self.vertex_index]

                # Reset PID controllers
                self.position_pid.reset()
                self.angle_pid.reset()

                # If we've completed a full hexagon and returned to start, stop
                if self.vertex_index == 0:
                    # Check if we're close to the starting position
                    start_vertex = self.vertices[0]
                    dist_to_start = math.sqrt(
                        (self.current_pose.x - start_vertex[0])**2 +
                        (self.current_pose.y - start_vertex[1])**2
                    )
                    if dist_to_start < self.position_threshold:
                        self.goal_reached = True
                        rospy.loginfo("Hexagon trajectory completed!")

            self.control_rate.sleep()

        # Stop the turtle
        self.stop_turtle()
        rospy.loginfo("Closed-loop control finished")

    def stop_turtle(self):
        """Stop the turtle by publishing zero velocities"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


if __name__ == '__main__':
    try:
        controller = ClosedLoopControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass