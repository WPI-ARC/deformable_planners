#!/usr/bin/env python

#   Calder Phillips-Grafflin - WPI/ARC Lab

import rospy
import math
from std_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from lightweight_vicon_bridge.msg import *

class MocapMarkerPublisher:

    def __init__(self, root_frame, rate):
        self.root_frame = root_frame
        self.rate = rate
        self.marker_pub = rospy.Publisher("mocap_markers_display", Marker)
        self.marker_sub = rospy.Subscriber("mocap_markers", MocapMarkerArray, self.marker_cb)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

    def marker_cb(self, msg):
        # Make the display message representation
        marker_msg = Marker()
        marker_msg.type = Marker.SPHERE_LIST
        marker_msg.ns = "mocap_markers"
        marker_msg.id = 1
        marker_msg.action = Marker.ADD
        marker_msg.lifetime = rospy.Duration(0.0)
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = self.root_frame
        marker_msg.scale.x = 0.014
        marker_msg.scale.y = 0.014
        marker_msg.scale.z = 0.014
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
        marker_color = ColorRGBA()
        marker_color.a = 1.0
        marker_color.b = 1.0
        marker_color.g = 1.0
        marker_color.r = 1.0
        # Make the individual points
        for marker in msg.markers:
            marker_msg.points.append(marker.position)
            marker_msg.colors.append(marker_color)
        self.marker_pub.publish(marker_msg)

if __name__ == "__main__":
    rospy.init_node("mocap_marker_publisher")
    rospy.loginfo("Starting the mocap marker broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "mocap_world")
    rate = rospy.get_param("~rate", 10.0)
    MocapMarkerPublisher(root_frame, rate)
