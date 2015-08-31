#!/usr/bin/python

import rospy
import numpy
import threading
from sensor_msgs.msg import *
from cv_bridge import *
from deformable_astar.srv import *


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    QUESTION = '\033[90m'
    FAIL = '\033[91m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    ENDC = '\033[0m'


class CostClient:

    CALIBRATED = 1
    CALIBRATING = 0
    NOT_CALIBRATED = -1

    def __init__(self, in_image_topic):
        self.bridge = CvBridge()
        self.calibration_status = self.NOT_CALIBRATED
        self.calibration_queue = []
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.calibrated_visible_area = 0.0
        self.total_deformation = 0.0
        rospy.Subscriber(in_image_topic, Image, self.image_cb)
        self.CalibrationRequestHandler = rospy.Service("calibrate_cost", CalibrateCost, self.calibrate_cb)
        self.CostRequestHandler = rospy.Service("cost_query", CostQuery, self.cost_cb)
        while not rospy.is_shutdown():
            rospy.spin()

    def cost_cb(self, request):
        if self.calibration_status == self.CALIBRATED:
            if request.reset_total_deformation:
                self.total_deformation = 0.0
                rospy.logwarn("Resetting total deformation to zero")
            with self.image_lock:
                working_image = self.latest_image
                self.latest_image = None
            if working_image is not None:
                current_visible_area = self.compute_visible_area(working_image)
                current_deformation = self.calibrated_visible_area - current_visible_area
                self.total_deformation += current_deformation
                response = CostQueryResponse()
                response.current_deformation = current_deformation
                response.total_deformation = self.total_deformation
                response.current_visible_area = current_visible_area
                return response
            else:
                rospy.logerr("Received cost query but no images available to use")
                response = CostQueryResponse()
                response.current_deformation = float('nan')
                response.total_deformation = float('nan')
                response.current_visible_area = float('nan')
                return response
        else:
            rospy.logerr("Received cost query before first calibration")
            response = CostQueryResponse()
            response.current_deformation = float('nan')
            response.total_deformation = float('nan')
            response.current_visible_area = float('nan')
            return response

    def calibrate_cb(self, request):
        # Start collecting frames for calibration
        self.calibration_status = self.CALIBRATING
        rospy.loginfo("Collecting " + str(request.num_frames) + " frames for calibration")
        while len(self.calibration_queue) < request.num_frames:
            pass
        # Stop collecting frames
        self.calibration_status = self.NOT_CALIBRATED
        # Calibrate
        average_visible_area = 0.0
        for frame in self.calibration_queue:
            average_visible_area += self.compute_visible_area(frame)
        average_visible_area /= float(len(self.calibration_queue))
        self.calibrated_visible_area = average_visible_area
        self.calibration_status = self.CALIBRATED
        # Flush the frames we collected
        self.calibration_queue = []
        rospy.loginfo("Calibrated with an average visible area of " + str(self.calibrated_visible_area))
        response = CalibrateCostResponse()
        response.average_visible_area = self.calibrated_visible_area
        return response

    def image_cb(self, msg):
        with self.image_lock:
            # Save the current binary image
            self.latest_image = self.bridge.imgmsg_to_cv2(msg)
            if self.calibration_status == self.CALIBRATING:
                self.calibration_queue.append(self.latest_image)

    def compute_visible_area(self, working_image):
        return float(numpy.sum(working_image) / 255)

if __name__ == '__main__':
    rospy.init_node('cost_client')
    print bcolors.HEADER + bcolors.BOLD + "[COST CLIENT] Starting the cost client..." + bcolors.ENDC
    #Get the parameters from the server
    image_topic = rospy.get_param("~binary_image_topic", "test_camera/binary_filtered")
    #Print the starting parameters to the screen
    print bcolors.HEADER + bcolors.BOLD + "[COST CLIENT] Starting with image topic: /" + image_topic + bcolors.ENDC
    CostClient(image_topic)
