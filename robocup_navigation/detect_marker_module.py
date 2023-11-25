#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point, Twist
from visualization_msgs.msg import Marker
from cv_bridge              import CvBridge, CvBridgeError
import robocup_navigation.process_image as proc
import time

class DetectMarker(Node):
    def __init__(self):
        super().__init__('detect_marker')
        self.get_logger().info('Looking for the marker...')
        self.image_sub = self.create_subscription(Image,"/image_in",self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_pub = self.create_publisher(Image, "/image_out", 1)
        self.aruco_pub = self.create_publisher(Marker,"/marker_aruco", 1)
        self.pos_pub = self.create_publisher(Point, '/marker_pos', 10)
        self.rot_pub = self.create_publisher(Point, '/marker_rot', 10)

        self.bridge = CvBridge()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.get_logger().info('Image Recieved')
            
        except CvBridgeError as e:
                print(e)
    
        try:
           corners = proc.find_marker(cv_image)
           if corners != 0:
                tvecs, rvecs = proc.marker_pos(corners)

                point_out = Point()
                rot_out = Point()

                point_out.x = tvecs[0][0][0]
                point_out.y = tvecs[0][0][1]
                point_out.z = tvecs[0][0][2]

                rot_out.x = rvecs[0][0][0]
                rot_out.y = rvecs[0][0][1]
                rot_out.z = rvecs[0][0][2]

                self.get_logger().info(f"Pt: ({point_out.x},{point_out.y},{point_out.z})")
                self.get_logger().info(f"Rot: ({rot_out.x},{rot_out.y},{rot_out.z})")
                
                self.pos_pub.publish(point_out)
                self.rot_pub.publish(rot_out)
                
            #print("no marker detected")
        
            #image = proc.find_marker(cv_image)

           """ img_to_pub = self.bridge.cv2_to_imgmsg(image, "bgr8")
           img_to_pub.header = data.header
           self.image_pub.publish(img_to_pub) """

           """ m = Marker()
           m.header.frame_id = "camera_link_optical"
           m.id = 0
           m.type = Marker.SPHERE
           m.action = Marker.ADD
           m.pose.position.x = tvecs[0][0][0]
           m.pose.position.y = tvecs[0][0][1]
           m.pose.position.z = tvecs[0][0][2]
           m.scale.x = 0.033*2
           m.scale.y = 0.033*2
           m.scale.z = 0.033*2
           m.color.r = 0.933
           m.color.g = 1.0
           m.color.b = 0.0
           m.color.a = 1.0
           self.aruco_pub.publish(m) """

        except CvBridgeError as e:
                print("An error has occured detecting the marker" + e)
