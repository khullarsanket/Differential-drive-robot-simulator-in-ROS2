#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import random
from time import time
from geometry_msgs.msg import Quaternion
from rclpy.time import Time
from rclpy.duration import Duration
from project4a.disc_robot import load_disc_robot


class DifferentialDriveSimulator(Node):
    def __init__(self,robot_name):
        super().__init__('differential_drive_simulator')
        self.robot = load_disc_robot(robot_name)
        # Robot parameters
        self.radius = self.robot['body']['radius']
        self.wheel_distance = self.robot['wheels']['distance']
        self.error_variance_left = self.robot['wheels']['error_variance_left']
        self.error_variance_right = self.robot['wheels']['error_variance_right']
        self.error_update_rate = self.robot['wheels']['error_update_rate']
        # print(f'{self.error_update_rate = }')
        self.last_command_time = self.get_clock().now()
        
        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocity
        self.vl = 0.0
        self.vr = 0.0
        
        # Error
        self.left_error = 1.0
        self.right_error = 1.0
        self.last_error_update_time = self.get_clock().now()

        
        # Subscribers
        self.subscription_vl = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.subscription_vr = self.create_subscription(Float64, '/vr', self.vr_callback, 10)

        # Transform broadcaster
        self.br = TransformBroadcaster(self)
        
        # Timer
        self.timer = self.create_timer(0.1, self.update_pose)

    def vl_callback(self, msg):
        self.vl = msg.data
        self.last_command_time = self.get_clock().now()  # Update the timestamp when a command is received

    def vr_callback(self, msg):
        self.vr = msg.data
        self.last_command_time = self.get_clock().now()  # Update the timestamp when a command is received
        
    def update_error(self):
        current_time = self.get_clock().now()
        # Check if more than one second has passed since the last command
        if (current_time - self.last_command_time) > Duration(seconds=1.0):
            self.vl = 0.0
            self.vr = 0.0

        if current_time - self.last_error_update_time > Duration(seconds=self.error_update_rate):
            self.left_error = np.random.normal(1, math.sqrt(self.error_variance_left))
            self.right_error = np.random.normal(1, math.sqrt(self.error_variance_right))
            self.last_error_update_time = current_time
    
    def update_pose(self):
        self.update_error()
        
        vl = self.vl * self.left_error
        vr = self.vr * self.right_error
        
        v = (vl + vr) / 2
        omega = (vr - vl) / self.wheel_distance
        
        delta_t = 0.1  # update interval
        delta_x = v * math.cos(self.theta) * delta_t
        delta_y = v * math.sin(self.theta) * delta_t
        delta_theta = omega * delta_t
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(0, 0, self.theta)
        
        self.br.sendTransform(t)


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx,y=qy,z=qz,w=qw)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('temp_node_for_param_retrieval')
    node.declare_parameter("robot_name",'/home/lab2004/project_ws/src/project4a/project4a/normal.robot')
    robot_name = node.get_parameter("robot_name").value
    print(f'in diff drive node the robot name is {robot_name}')
    node.destroy_node()



    differential_drive_simulator = DifferentialDriveSimulator(robot_name)
    rclpy.spin(differential_drive_simulator)
    differential_drive_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
