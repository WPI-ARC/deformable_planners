#!/usr/bin/python

import rospy
import cv
import cv2
import time
import subprocess
import numpy
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import *
from deformable_astar.msg import *
from deformable_astar.srv import *
from transformation_helper import *
from image_sdf.srv import *

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

class CameraClient:

    def __init__(self, image_topic, image_width, image_height, threshold, data_path):
        self.image_width = image_width
        self.image_height = image_height
        self.data_path = data_path
        self.threshold = threshold
        rospy.init_node('camera_client')
        self.RequestHandler = rospy.Service("camera_client/CameraTrigger", CameraTrigger, self.RequestHandler)
        self.sdf_client = rospy.ServiceProxy("compute_gradient", ComputeGradient)
        self.bridge = CvBridge()
        rospy.Subscriber(image_topic, Image, self.image_cb)
        self.calibrated = False
        self.reference_image_opencv = None
        self.reference_binary = None
        self.calibration = None
        self.calibrated_mask = None
        self.calibrated_corners = [None, None, None, None]
        self.latest_image = None
        self.total_deformation = 0.0
        cv.NamedWindow("calibration")
        cv.NamedWindow("deformation tracker")
        while not rospy.is_shutdown():
            rospy.spin()

    def RequestHandler(self, request):
        if (self.calibrated and self.latest_image is not None):
            if (request.reset == 1):
                self.total_deformation = 0.0
            print bcolors.CYAN + "[CAMERA CLIENT] Using current calibration - " + str(self.calibration) + bcolors.ENDC
            print bcolors.QUESTION + "[CAMERA CLIENT] Ready to process deformation" + bcolors.ENDC
            masked = self.apply_mask(self.latest_image)
            '''
            binary = self.convert_to_binary(masked)
            [xored, deformation] = self.compute_deformation(self.reference_binary, binary)
            '''
            [cropped, deformation] = self.count_green_pixels(masked)
            info_str = "Measured deformation: " + str(deformation)
            img_font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, thickness=2)
            cv.PutText(cropped, info_str, (50,50), img_font, (255,255,255))
            cv.ShowImage("deformation tracker", cropped)
            # This is how you get OpenCV to actually draw the window
            # (otherwise, it never has time to process)
            cv.WaitKey(3)
            response = CameraTriggerResponse()
            response.status = "CALIBRATED"
            response.deformation = deformation
            self.total_deformation += deformation
            response.total_deformation = self.total_deformation
            self.latest_image = None
            return response
        else:
            response = CameraTriggerResponse()
            response.status = "NOT CALIBRATED"
            response.deformation = -1.0
            response.total_deformation = -1.0
            response.gradient.x = 0.0
            response.gradient.y = 0.0
            response.gradient.z = 0.0
            return response

    def image_cb(self, msg):
        # Process the image request
        if (self.calibrated):
            cv_image = self.bridge.imgmsg_to_cv(msg, "bgr8")
            self.latest_image = cv_image
            #print bcolors.OKBLUE + "[CAMERA CLIENT] Got a new image and converted to OpenCV" + bcolors.ENDC
            #print bcolors.CYAN + "[CAMERA CLIENT] Using current calibration - " + str(self.calibration) + bcolors.ENDC
            #print bcolors.QUESTION + "[CAMERA CLIENT] Ready to process deformation" + bcolors.ENDC
            #masked = self.apply_mask(cv_image)
            #binary = self.convert_to_binary(masked)
            #deformation = self.compute_deformation(self.reference_binary, binary)
            #info_str = "Measured deformation: " + str(deformation)
            #img_font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, thickness=2)
            #cv.PutText(binary, info_str, (50,50), img_font, (0,0,0))
            #cv.ShowImage("deformation tracker", binary)
            # This is how you get OpenCV to actually draw the window
            # (otherwise, it never has time to process)
            cv.WaitKey(3)
        else:
            cv_image = self.bridge.imgmsg_to_cv(msg, "bgr8")
            self.calibrate(cv_image)

    def compute_deformation(self, reference_image, current_image):
        xored = cv.CreateMat(current_image.rows, current_image.cols, current_image.type)
        cv.Xor(reference_image, current_image, xored)
        numpyed = numpy.asarray(xored)
        deformation = self.sum_pixels(numpyed)
        return [xored, deformation]

    def sum_pixels(self, pixel_diff):
        return numpy.sum(pixel_diff) / 255

    def crop_to_color_scalar(self, cv_image, lower, upper):
        binary = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC1)
        cv.InRangeS(cv_image, lower, upper, binary)
        return binary

    def crop_to_color_array(self, cv_image, lower, upper):
        binary = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC1)
        cv.InRange(cv_image, lower, upper, binary)
        return binary

    def count_red_pixels(self, cv_image):
        #REMEMBER, THIS IS BGR!
        red_lower = cv.Scalar(0, 0, 150)
        red_upper = cv.Scalar(100, 100, 255)
        cropped = self.crop_to_color_scalar(cv_image, red_lower, red_upper)
        count = self.sum_pixels(cropped)
        return [cropped, count]

    def count_green_pixels(self, cv_image):
        #REMEMBER, THIS IS BGR!
        green_lower = cv.Scalar(0, 60, 0)
        green_upper = cv.Scalar(120, 150, 120)
        cropped = self.crop_to_color_scalar(cv_image, green_lower, green_upper)
        #filter
        filtered = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC1)
        mean_kernel = cv.CreateMat(1,1,cv.CV_32FC1)
        cv.Set(mean_kernel, 1.0)
        cv.Filter2D(cropped, filtered, mean_kernel)
        count = self.sum_pixels(filtered)
        return [filtered, count]        

    def convert_to_binary(self, cv_image):
        grayscale = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC1)
        cv.CvtColor(cv_image, grayscale, cv.CV_RGB2GRAY)
        binary = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC1)
        cv.Threshold(grayscale, binary, self.threshold, 255, cv.CV_THRESH_BINARY)
        filtered = binary
        #filtered = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC1)
        #mean_kernel = cv.CreateMat(9,9,cv.CV_32FC1)
        #cv.Set(mean_kernel, 1.0)
        ##mexican_numpy = numpy.array([[0,0,-1,-1,-1,0,0],[0,-1,-3,-3,-3,-1,0],[-1,-3,0,7,0,-3,-1],[-1,-3,7,24,7,-3,-1],[-1,-3,0,7,0,-3,-1],[0,-1,-3,-3,-3,-1,0],[0,0,-1,-1,-1,0,0]],dtype=numpy.float32)
        ##mexican_kernel = cv.fromarray(mexican_numpy)
        #cv.Filter2D(binary, filtered, mean_kernel)
        ##filtered = numpy.asarray(binary)
        ##cv2.medianBlur(filtered,5)
        ##filtered = cv.fromarray(filtered)
        '''
        element = cv.CreateMat()
        eroded = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC1)
        cv.erode(binary, eroded, element)
        dilated = cv.CreateMat(cv_image.rows, cv_image.cols, cv.CV_8UC1)
        cv.dilate(eroded, dilated, element)
        filtered = dilated
        '''
        if (self.reference_binary == None):
            reference_copy = cv.CreateMat(cv_image.rows, cv_image.cols, filtered.type)
            cv.Copy(filtered, reference_copy)
            self.reference_binary = reference_copy
        return filtered

    def apply_mask(self, cv_image):
        destination = cv.CreateMat(cv_image.rows, cv_image.cols, cv_image.type)
        cv.Set(destination, (255,0,255))
        cv.Copy(cv_image, destination, self.calibrated_mask)
        return destination

    def make_polygon_from_center(self, width, height, center):
        c1 = [center[0] - (width / 2), center[1] - (height / 2)]
        c2 = [center[0] + (width / 2), center[1] - (height / 2)]
        c3 = [center[0] - (width / 2), center[1] + (height / 2)]
        c4 = [center[0] + (width / 2), center[1] + (height / 2)]
        return [c1, c2, c4, c3]

    def generate_mask(self, reference_image, corners):
        mask = cv.CreateMat(reference_image.rows, reference_image.cols, cv.CV_8UC1)
        cv.Set(mask, 0)
        #Make corner masks to mask out the markers
        cpoly1 = self.make_polygon_from_center(200, 200, corners[0])
        cpoly2 = self.make_polygon_from_center(200, 200, corners[1])
        cpoly3 = self.make_polygon_from_center(200, 200, corners[2])
        cpoly4 = self.make_polygon_from_center(200, 200, corners[3])
        #Mask out the areas outside the calibration markers
        for i in range(reference_image.cols):
            for j in range(reference_image.rows):
                if (self.in_polygon(i, j, corners)):
                    mask[j,i] = 1
                if (self.in_polygon(i, j, cpoly1) or self.in_polygon(i, j, cpoly2) or self.in_polygon(i, j, cpoly3) or self.in_polygon(i, j, cpoly4)):
                    mask[j,i] = 0
        return mask

    def in_polygon(self, x, y, poly):
        n = len(poly)
        inside =False
        p1x,p1y = poly[0]
        for i in range(n+1):
            p2x,p2y = poly[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x,p1y = p2x,p2y
        return inside

    def calibrate(self, image):
        # Attempt to calibrate from the image
        successful = False
        centers = self.find_markers(image)
        if (centers[0] is not None and centers[1] is not None and centers[2] is not None and centers[3] is not None):
            successful = True
        if (successful):
            self.calibrated = True
            self.calibration = centers
            self.calibrated_corners = centers
            self.reference_image_opencv = image
            self.calibrated_mask = self.generate_mask(image, centers)
            masked = self.apply_mask(image)
            self.convert_to_binary(masked)
            save_path = self.data_path + time.strftime("/deformation_%d-%m-%Y_%H-%M-%S_reference.png")
            cv.SaveImage(save_path, image)
            save_path2 = self.data_path + time.strftime("/deformation_%d-%m-%Y_%H-%M-%S_reference_masked.png")
            [cropped, deformation] = self.count_green_pixels(masked)
            cv.SaveImage(save_path2, masked)
            save_path3 = self.data_path + time.strftime("/deformation_%d-%m-%Y_%H-%M-%S_reference_colorcropped.png")
            cv.SaveImage(save_path3, cropped)
            cv.DestroyWindow("calibration")
            cv.WaitKey(3)
            print bcolors.OKGREEN + bcolors.BOLD + "[CAMERA CLIENT] Calibration attempt successful - Saved a reference image for this session" + bcolors.ENDC
        else:
            print bcolors.WARNING + "[CAMERA CLIENT] Calibration attempt unsuccessful" + bcolors.ENDC

    def find_checkerboard_center(self, corners, offset):
        try:
            x_cnt = 0
            y_cnt = 0
            cnt = len(corners)
            for corner in corners:
                x_cnt += corner[0]
                y_cnt += corner[1]
            x_avg = x_cnt / cnt
            y_avg = y_cnt / cnt
            return (int(x_avg)+offset[0], int(y_avg)+offset[1])
        except:
            return (0+offset[0], 0+offset[1])

    def process_corners(self, cv_image, pattern_size, offset):
        try:
            corners = cv.FindChessboardCorners(cv_image, pattern_size)
            cv.DrawChessboardCorners(cv_image, pattern_size, corners[1], corners[0])
            return self.find_checkerboard_center(corners[1], offset)
        except:
            print bcolors.FAIL + bcolors.BOLD + "[CAMERA CLIENT] *** UNABLE TO FIND CHECKERBOARD IN THE IMAGE ***" + bcolors.ENDC
            return None

    def find_markers(self, cv_image):
        pattern_size = (3,3)
        # Find the corners of the primary chessboard pattern
        LTI = cv.GetSubRect(cv_image, (0,0,600,600))
        LT = self.process_corners(LTI, pattern_size, (0,0))
        RTI = cv.GetSubRect(cv_image, (1320,0,600,600))
        RT = self.process_corners(RTI, pattern_size, (1320,0))
        LBI = cv.GetSubRect(cv_image, (0,480,600,600))
        LB = self.process_corners(LBI, pattern_size, (0,480))
        RBI = cv.GetSubRect(cv_image, (1320,480,600,600))
        RB = self.process_corners(RBI, pattern_size, (1320,480))
        #corners = cv.FindChessboardCorners(cv_image, (5,5))
        #cv.DrawChessboardCorners(cv_image, (5,5), corners[1], corners[0])
        # Draw the marker-to-marker lines in the image
        #LT = (100,100)
        #RT = (1820,100)
        #LB = (100,980)
        #RB = (1820,980)
        print LT
        print RT
        print LB
        print RB
        try:
            magenta = cv.RGB(255,0,255)
            cv.Line(cv_image, LT, RT, magenta)
            cv.Line(cv_image, RT, RB, magenta)
            cv.Line(cv_image, RB, LB, magenta)
            cv.Line(cv_image, LB, LT, magenta)
            # Display the image to the screen
            cv.ShowImage("calibration", cv_image)
            # This is how you get OpenCV to actually draw the window
            # (otherwise, it never has time to process)
            cv.WaitKey(3)
            #print corners
        except:
            cv.ShowImage("calibration", cv_image)
            # This is how you get OpenCV to actually draw the window
            # (otherwise, it never has time to process)
            cv.WaitKey(3)
        return [LT,RT,RB,LB]

if __name__ == '__main__':
    path = subprocess.check_output("rospack find deformable_astar", shell=True)
    path = path.strip("\n") + "/data"
    print bcolors.HEADER + bcolors.BOLD + "[CAMERA CLIENT] Starting the camera client..." + bcolors.ENDC
    #Get the parameters from the server
    image_topic = rospy.get_param("camera_client/image_source", "logitech_hd_cam/image")
    image_width = rospy.get_param("camera_client/image_width", 1920)
    image_height = rospy.get_param("camera_client/image_height", 1080)
    foam_threshold = rospy.get_param("camera_client/threshold", 75)
    #Print the starting parameters to the screen
    print bcolors.HEADER + bcolors.BOLD + "[CAMERA CLIENT] Starting with image topic: /" + image_topic + bcolors.ENDC
    print bcolors.HEADER + bcolors.BOLD + "[CAMERA CLIENT] Starting with image width: " + str(image_width) + bcolors.ENDC
    print bcolors.HEADER + bcolors.BOLD + "[CAMERA CLIENT] Starting with image height: " + str(image_height) + bcolors.ENDC
    print bcolors.HEADER + bcolors.BOLD + "[CAMERA CLIENT] Starting with threshold: " + str(foam_threshold) + bcolors.ENDC
    CameraClient(image_topic, image_width, image_height, foam_threshold, path)
