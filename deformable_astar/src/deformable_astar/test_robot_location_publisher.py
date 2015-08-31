#!/usr/bin/env python

#   Calder Phillips-Grafflin - WPI/ARC Lab

import rospy
import math
import threading
from tf.transformations import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from transformation_helper import *

class TestRobotLocationPublisher:

    def __init__(self, centers_topic, markers_topic, tracking_topic):
        self.centers_lock = threading.Lock()
        self.latest_centers = None
        self.centers_sub = rospy.Subscriber(centers_topic, Int32MultiArray, self.centers_cb)
        self.markers_sub = rospy.Subscriber(markers_topic, Int32MultiArray, self.markers_cb)
        self.tracking_pub = rospy.Publisher(tracking_topic, Vector3)
        while not rospy.is_shutdown():
            rospy.spin()

    def centers_cb(self, msg):
        new_centers = []
        for cidx in xrange(msg.layout.dim[0].size):
            new_center = []
            for eidx in xrange(msg.layout.dim[1].size):
                new_center.append(msg.data[msg.layout.data_offset + (msg.layout.dim[1].stride * cidx) + eidx])
            new_centers.append(new_center)
        with self.centers_lock:
            self.latest_centers = new_centers

    def markers_cb(self, msg):
        # Get the tracked corners
        centers = None
        with self.centers_lock:
            centers = self.latest_centers
        if (centers == None):
            rospy.logerr("No available centers, cannot compute robot location")
        else:
            # Extract the (x,y) pairs from the tracking message
            marker_centers = []
            for cidx in xrange(msg.layout.dim[0].size):
                marker_center = []
                for eidx in xrange(msg.layout.dim[1].size):
                    marker_center.append(msg.data[msg.layout.data_offset + (msg.layout.dim[1].stride * cidx) + eidx])
                marker_centers.append(marker_center)
            marker_1_center = marker_centers[0]
            marker_2_center = marker_centers[1]
            # Compute pixels/environment cell
            horz_pixels = ((centers[1][0] - centers[0][0]) + (centers[2][0] - centers[3][0])) / 2.0
            vert_pixels = ((centers[3][1] - centers[0][1]) + (centers[2][1] - centers[1][1])) / 2.0
            horz_cells = 95.0
            vert_cells = 81.0
            horz_pixels_per_cell = horz_pixels / horz_cells
            vert_pixels_per_cell = vert_pixels / vert_cells
            originH = int(round(centers[0][0] - (10 * horz_pixels_per_cell)))
            originV = int(round(centers[0][1] - (1 * vert_pixels_per_cell)))
            # Compute the rotation of the robot
            delta_pixX = marker_1_center[0] - marker_2_center[0]
            delta_pixY = marker_1_center[1] - marker_2_center[1]
            angle = math.atan2(delta_pixX, delta_pixY) - (math.pi / 2.0)
            # Since marker 2 is over CoR, convert its center to Cells
            center_X = (marker_2_center[1] - originV) / vert_pixels_per_cell
            center_Y = (marker_2_center[0] - originH) / horz_pixels_per_cell
            # Convert to Vector3
            tracking_msg = Vector3()
            tracking_msg.x = center_X
            tracking_msg.y = center_Y
            tracking_msg.z = angle
            # Publish track
            self.tracking_pub.publish(tracking_msg)

if __name__ == "__main__":
    rospy.init_node("test_robot_location_publisher")
    rospy.loginfo("Starting the test robot location broadcaster...")
    centers_topic = rospy.get_param("~centers_topic", "test_camera/centers")
    markers_topic = rospy.get_param("~markers_topic", "test_camera/markers")
    tracking_topic = rospy.get_param("~tracking_topic", "test_camera/tracking")
    TestRobotLocationPublisher(centers_topic, markers_topic, tracking_topic)
