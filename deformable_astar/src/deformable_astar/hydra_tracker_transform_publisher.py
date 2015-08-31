#!/usr/bin/env python

#   Calder Phillips-Grafflin - WPI/ARC Lab

import rospy
import math
import tf
from tf.transformations import *
from sensor_msgs.msg import *
from razer_hydra.msg import *
from transformation_helper import *

class HydraTrackerTransformPublisher:

    LEFT_PADDLE = 0
    RIGHT_PADDLE = 1

    def __init__(self, target_frame, root_frame, hydra_topic, rate):
        self.target_frame = target_frame
        self.root_frame = root_frame
        self.rate = rate
        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.rz = 0.0
        self.rw = 1.0
        self.broadcaster = tf.TransformBroadcaster()
        self.hydra_sub = rospy.Subscriber(hydra_topic, Hydra, self.hydra_cb)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform((self.tx, self.ty, self.tz), (self.rx, self.ry, self.rz, self.rw), rospy.Time.now(), self.target_frame, self.root_frame)
            rate.sleep()

    def hydra_cb(self, msg):
        left_paddle_transform = msg.paddles[self.LEFT_PADDLE].transform
        inv_left_paddle_transform = InvertTransform(left_paddle_transform)
        self.tx = inv_left_paddle_transform.translation.x
        self.ty = inv_left_paddle_transform.translation.y
        self.tz = inv_left_paddle_transform.translation.z
        self.rx = inv_left_paddle_transform.rotation.x
        self.ry = inv_left_paddle_transform.rotation.y
        self.rz = inv_left_paddle_transform.rotation.z
        self.rw = inv_left_paddle_transform.rotation.w

if __name__ == "__main__":
    rospy.init_node("hydra_tracker_transform_publisher")
    rospy.loginfo("Starting the hydra tracker transform broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "hydra_bracket")
    target_frame = rospy.get_param("~target_frame", "hydra_base")
    hydra_topic = rospy.get_param("~hydra_topic", "hydra_calib")
    rate = rospy.get_param("~rate", 100.0)
    HydraTrackerTransformPublisher(target_frame, root_frame, hydra_topic, rate)
