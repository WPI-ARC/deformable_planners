#!/usr/bin/env python

#   Calder Phillips-Grafflin - WPI/ARC Lab

import rospy
import math
import tf
from tf.transformations import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *

class RobotMarkerPublisher:

    def __init__(self, root_frame, rate):
        self.root_frame = root_frame
        self.rate = rate
        self.marker_pub = rospy.Publisher("robot_markers", Marker)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.display_table()
            rate.sleep()

    def display_table(self):
        # Make table top
        marker_msg = Marker()
        marker_msg.type = Marker.CUBE_LIST
        marker_msg.ns = "robot"
        marker_msg.id = 1
        marker_msg.action = Marker.ADD
        marker_msg.lifetime = rospy.Duration(0.0)
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = self.root_frame
        marker_msg.scale.x = 0.04
        marker_msg.scale.y = 0.04
        marker_msg.scale.z = 0.02
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.g = 0.0
        marker_msg.pose.position.x = 0.0
        marker_msg.pose.position.y = 0.0
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0
        # Make the individual points
        p1 = Point()
        p1.x = 0.001
        p1.y = -0.002
        p1.z = -0.01
        p2 = Point()
        p2.x = p1.x
        p2.y = p1.y + 0.04
        p2.z = p1.z
        p3 = Point()
        p3.x = p1.x - 0.04
        p3.y = p1.y
        p3.z = p1.z
        marker_msg.points = [p1, p2, p3]
        marker_msg.colors = [marker_msg.color, marker_msg.color, marker_msg.color]
        self.marker_pub.publish(marker_msg)

if __name__ == "__main__":
    rospy.init_node("robot_marker_publisher")
    rospy.loginfo("Starting the robot marker broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "test_robot_bracket_frame")
    rate = rospy.get_param("~rate", 10.0)
    RobotMarkerPublisher(root_frame, rate)
