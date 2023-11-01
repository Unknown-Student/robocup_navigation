#!/usr/bin/env python3

import rclpy
from robocup_navigation.module_to_include import DrawCircleNode

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()