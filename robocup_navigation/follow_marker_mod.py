#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point, Twist
import time

class FollowMarker(Node):
    def __init__(self):
        super().__init__('follow_marker')
        self.get_logger().info('Follow Marker started')
        self.sub_ = self.create_subscription(Point,"/marker_pos",self.listener,10)
        self.pub_ = self.create_publisher(Twist,"/cmd_vel",10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.target_val = 0.0
        self.target_dist = 0.0
        self.target_ang = 0.0
        self.thresh_dist = 0.2

    def listener(self, data):
        filter_val = 0.9
        self.target_val = self.target_val * filter_val + data.x *(1-filter_val)     
        self.target_dist = self.target_dist * filter_val + data.y * (1-filter_val)
        self.target_ang = self.target_ang * filter_val + data.z * (1-filter_val)

    def timer_callback(self):
        msg = Twist()
        if (self.target_dist > self.thresh_dist):
            msg.linear.x = 0.1
        msg.linear.y =  0.7 * self.target_ang
        msg.angular.z = - 0.7 * self.target_val
        self.get_logger().info(f"Vel_x: {msg.linear.x}, Vel_y: {msg.linear.y}, Ang_vel: {msg.angular.z}")
        self.pub_.publish(msg)