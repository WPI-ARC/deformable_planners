#!/usr/bin/python

import rospy
import cv
import cv2
import time
import subprocess
import numpy
import threading
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import *
from deformable_astar.msg import *
from deformable_astar.srv import *
from transformation_helper import *
from xtf.xtf import *
from grc import *

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

class SDFClient:

    def __init__(self, sdf_topic, centers_topic, sdf_threshold, gradient_multiplier):
        rospy.init_node('sdf_client')
        # Set the dimensions of the environment
        # Horizontal (Y) is 0.55m (55cm)
        # Each cell is 5mm x 5mm, so we have 110.0 horizontal cells
        self.horz_cells = 110.0
        # Vertical (X) is o.402m (40.2cm)
        # Each cell is 5mm x 5mm, so we have 80.4 vertical cells
        self.vert_cells = 80.4
        # Set the offset between the upper left checkerboard center and the origin cell of the environment
        # Horizontal (Y) is 1cm
        self.origin_horz_cell_offset = -2.5
        # Vertical (X) is 1cm
        self.origin_vert_cell_offset = -0.5
        self.sdf_threshold = sdf_threshold
        self.gradient_multiplier = gradient_multiplier
        self.sdf_request_srv = rospy.Service("sdf_query", SDFQuery, self.sdf_request_cb)
        self.grc_request_srv = rospy.Service("grc_query", GRCQuery, self.grc_request_cb)
        self.bridge = CvBridge()
        self.controller = GradientRejectionController()
        self.sdf_lock = threading.Lock()
        self.latest_sdf = None
        self.latest_centers = None
        cv.NamedWindow("Status")
        # Load the trajectory file
        parser = XTFParser()
        self.path = parser.ParseTraj("/home/calderpg/Dropbox/Classes/AI/Project/code/TestEnv2.xtf")
        self.sdf_sub = rospy.Subscriber(sdf_topic, Image, self.sdf_cb)
        self.centers_sub = rospy.Subscriber(centers_topic, Int32MultiArray, self.centers_cb)
        while not rospy.is_shutdown():
            rospy.spin()

    def grc_request_cb(self, request):
        # Lock the SDF representation
        with self.sdf_lock:
            if (self.latest_sdf == None or self.latest_centers == None):
                rospy.logerr("Unable to complete request as no SDF or centers have been received")
                response = GRCQueryResponse()
                response.corrected_target.x = request.current_target.x
                response.corrected_target.y = request.current_target.y
                response.corrected_target.z = request.current_target.z
                return response
            else:
                sdf = self.latest_sdf
                centers = self.latest_centers
        # Call the gradient computation
        [rendered_image, current_gradient, originH, originV, horz_pixels_per_cell, vert_pixels_per_cell, robot_voxels] = self.compute_current_gradient(sdf, centers, [request.current_state.x, request.current_state.y, request.current_state.z])
        # Draw the current 'target' state (center only)
        idealtargetcenterH = int(round(originH + (request.current_target.y * horz_pixels_per_cell)))
        idealtargetcenterV = int(round(originV + (request.current_target.x * vert_pixels_per_cell)))
        cv.Circle(rendered_image, (idealtargetcenterH,idealtargetcenterV), 3, (0,0,255), thickness=-1)
        # Draw the next 'target' state (center only)
        nexttargetcenterH = int(round(originH + (request.next_target.y * horz_pixels_per_cell)))
        nexttargetcenterV = int(round(originV + (request.next_target.x * vert_pixels_per_cell)))
        cv.Circle(rendered_image, (nexttargetcenterH,nexttargetcenterV), 3, (255,0,255), thickness=-1)
        # Compute the GRC corrections for the corrected target state
        # We're going to deal with translation and rotation separately here
        # First, let's handle translation
        current_ideal_np = numpy.array([request.current_ideal.x, request.current_ideal.y])
        current_state_np = numpy.array([request.current_state.x, request.current_state.y])
        current_target_np = numpy.array([request.current_target.x, request.current_target.y])
        next_target_np = numpy.array([request.next_target.x, request.next_target.y])
        current_translation_gradient = numpy.array([current_gradient[0], current_gradient[1]])
        # Send the various states+gradient to the GRC controller
        corrected_target_translation_np = self.controller.compute_new_target(current_ideal_np, current_state_np, current_target_np, next_target_np, current_translation_gradient, request.grc_control)
        # Second, let's handle rotation
        current_ideal_rotation = request.current_ideal.z
        current_state_rotation = request.current_state.z
        current_target_rotation = request.current_target.z
        next_target_rotation = request.next_target.z
        current_rotation_gradient = float(current_gradient[2])
        # If the current rotation gradient is above bound, cap it to 0.1
        rotation_gradient_bound = 0.1
        if (current_rotation_gradient > rotation_gradient_bound):
            current_rotation_gradient = rotation_gradient_bound
        elif (current_rotation_gradient < -rotation_gradient_bound):
            current_rotation_gradient = -rotation_gradient_bound
        print "Current rotation gradient " + str(current_rotation_gradient) + " (bounded)"
        # Simply add the current rotation gradient to the target rotation
        corrected_target_rotation = current_target_rotation + current_rotation_gradient
        # Now that we're done, bring translation and rotation back together
        corrected_target = [float(corrected_target_translation_np[0]), float(corrected_target_translation_np[1]), corrected_target_rotation]
        # Draw the corrected target state
        # First, the target center
        targetcenterH = int(round(originH + (corrected_target[1] * horz_pixels_per_cell)))
        targetcenterV = int(round(originV + (corrected_target[0] * vert_pixels_per_cell)))
        cv.Circle(rendered_image, (targetcenterH,targetcenterV), 3, (0,127,255), thickness=-1)
        # Transform each of the points in the robot into environment (x,y) locations
        target_environment_cells = []
        for (rbx, rby) in robot_voxels:
            [ex, ey] = self.transform(corrected_target[0], corrected_target[1], corrected_target[2], rbx, rby)
            target_environment_cells.append([ex, ey])
        # Convert the environment cells to pixels and draw
        for (ex, ey) in target_environment_cells:
            eH = int(round(originH + (ey * horz_pixels_per_cell)))
            eV = int(round(originV + (ex * vert_pixels_per_cell)))
            cv.Circle(rendered_image, (eH,eV), 3, (0,127,255))
        # Final steps
        # Resize the image to double size
        rendered_image_full_size = cv.CreateMat(1080, 1920, cv.CV_8UC3)
        cv.Resize(rendered_image, rendered_image_full_size)
        #cv.ShowImage("Status", sdf_false_color)
        cv.ShowImage("Status", rendered_image_full_size)
        # This is how you get OpenCV to actually draw the window
        # (otherwise, it never has time to process)
        cv.WaitKey(3)
        # Return the response
        response = GRCQueryResponse()
        response.corrected_target.x = corrected_target[0]
        response.corrected_target.y = corrected_target[1]
        response.corrected_target.z = corrected_target[2]
        return response

    def compute_current_gradient(self, current_sdf, current_corners, current_position):
        # Make the basic false-color SDF image for visualization
        sdf_false_color = cv.CreateMat(current_sdf.rows, current_sdf.cols, cv.CV_8UC3)
        max_distance = 100
        for i in xrange(current_sdf.rows):
            for j in xrange(current_sdf.cols):
                [posdist, negdist] = current_sdf[i, j]
                if negdist == 0.0:
                    blue_channel = 0
                    green_channel = 0
                    red_channel = int(64.0 + (64.0 * (posdist / max_distance)))
                    sdf_false_color[i, j] = [blue_channel, green_channel, red_channel]
                elif posdist == 0.0:
                    blue_channel = int(64.0 + (64.0 * (negdist / max_distance)))
                    green_channel = 0
                    red_channel = 0
                    sdf_false_color[i, j] = [blue_channel, green_channel, red_channel]
                else:
                    print "SDF image channels 1 and 2 not correctly separated"
        # Compute pixels/environment cell
        horz_pixels = ((current_corners[1][0] - current_corners[0][0]) + (current_corners[2][0] - current_corners[3][0])) / 2.0
        vert_pixels = ((current_corners[3][1] - current_corners[0][1]) + (current_corners[2][1] - current_corners[1][1])) / 2.0
        horz_pixels_per_cell = horz_pixels / self.horz_cells
        vert_pixels_per_cell = vert_pixels / self.vert_cells
        # Draw origin point
        originH = int(round(current_corners[0][0] + (self.origin_horz_cell_offset * horz_pixels_per_cell)))
        originV = int(round(current_corners[0][1] + (self.origin_vert_cell_offset * vert_pixels_per_cell)))
        originpoint1 = (originH - 5, originV - 5)
        originpoint2 = (originH + 5, originV + 5)
        cv.Rectangle(sdf_false_color, originpoint1, originpoint2, (255, 0, 255), thickness=-1)
        # Draw a calibration marker in the center of the image
        cal_centerH = int(round(current_corners[0][0] + ((self.horz_cells * 0.5) * horz_pixels_per_cell)))
        cal_centerV = int(round(current_corners[0][1] + ((self.vert_cells * 0.5) * vert_pixels_per_cell)))
        cal_centerpoint1 = (cal_centerH - 10, cal_centerV - 10)
        cal_centerpoint2 = (cal_centerH + 10, cal_centerV + 10)
        cv.Rectangle(sdf_false_color, cal_centerpoint1, cal_centerpoint2, (255, 255, 255), thickness=2)
        # Draw markers over the locations of the corner checkerboards
        ulccpoint1 = (current_corners[0][0] - 10, current_corners[0][1] - 10)
        ulccpoint2 = (current_corners[0][0] + 10, current_corners[0][1] + 10)
        cv.Rectangle(sdf_false_color, ulccpoint1, ulccpoint2, (0, 255, 255), thickness=2)
        urccpoint1 = (current_corners[1][0] - 10, current_corners[1][1] - 10)
        urccpoint2 = (current_corners[1][0] + 10, current_corners[1][1] + 10)
        cv.Rectangle(sdf_false_color, urccpoint1, urccpoint2, (0, 255, 255), thickness=2)
        lrccpoint1 = (current_corners[3][0] - 10, current_corners[3][1] - 10)
        lrccpoint2 = (current_corners[3][0] + 10, current_corners[3][1] + 10)
        cv.Rectangle(sdf_false_color, lrccpoint1, lrccpoint2, (0, 255, 255), thickness=2)
        llccpoint1 = (current_corners[2][0] - 10, current_corners[2][1] - 10)
        llccpoint2 = (current_corners[2][0] + 10, current_corners[2][1] + 10)
        cv.Rectangle(sdf_false_color, llccpoint1, llccpoint2, (0, 255, 255), thickness=2)
        # Draw markers where we *expected* the checkerboards to be!
        cal_ulcc_H = int(round(current_corners[0][0]))
        cal_ulcc_V = int(round(current_corners[0][1]))
        eulccpoint1 = (cal_ulcc_H - 10, cal_ulcc_V - 10)
        eulccpoint2 = (cal_ulcc_H + 10, cal_ulcc_V + 10)
        cv.Rectangle(sdf_false_color, eulccpoint1, eulccpoint2, (255, 255, 255), thickness=2)
        cal_urcc_H = int(round(current_corners[0][0] + (self.horz_cells * horz_pixels_per_cell)))
        cal_urcc_V = int(round(current_corners[0][1]))
        eurccpoint1 = (cal_urcc_H - 10, cal_urcc_V - 10)
        eurccpoint2 = (cal_urcc_H + 10, cal_urcc_V + 10)
        cv.Rectangle(sdf_false_color, eurccpoint1, eurccpoint2, (255, 255, 255), thickness=2)
        cal_llcc_H = int(round(current_corners[0][0]))
        cal_llcc_V = int(round(current_corners[0][1] + (self.vert_cells * vert_pixels_per_cell)))
        ellccpoint1 = (cal_llcc_H - 10, cal_llcc_V - 10)
        ellccpoint2 = (cal_llcc_H + 10, cal_llcc_V + 10)
        cv.Rectangle(sdf_false_color, ellccpoint1, ellccpoint2, (255, 255, 255), thickness=2)
        cal_lrcc_H = int(round(current_corners[0][0] + (self.horz_cells * horz_pixels_per_cell)))
        cal_lrcc_V = int(round(current_corners[0][1] + (self.vert_cells * vert_pixels_per_cell)))
        elrccpoint1 = (cal_lrcc_H - 10, cal_lrcc_V - 10)
        elrccpoint2 = (cal_lrcc_H + 10, cal_lrcc_V + 10)
        cv.Rectangle(sdf_false_color, elrccpoint1, elrccpoint2, (255, 255, 255), thickness=2)
        # Draw start point
        [startCellX, startCellY, startRotate] = self.path.trajectory[0].position_desired
        startH = int(round(originH + (startCellY * horz_pixels_per_cell)))
        startV = int(round(originV + (startCellX * vert_pixels_per_cell)))
        startpoint1 = (startH - 5, startV - 5)
        startpoint2 = (startH + 5, startV + 5)
        cv.Rectangle(sdf_false_color, startpoint1, startpoint2, (255, 0, 0), thickness=-1)
        # Draw goal point
        [goalCellX, goalCellY, goalRotate] = self.path.trajectory[-1].position_desired
        goalH = int(round(originH + (goalCellY * horz_pixels_per_cell)))
        goalV = int(round(originV + (goalCellX * vert_pixels_per_cell)))
        goalpoint1 = (goalH - 5, goalV - 5)
        goalpoint2 = (goalH + 5, goalV + 5)
        cv.Rectangle(sdf_false_color, goalpoint1, goalpoint2, (0, 255, 0), thickness=-1)
        # Render the trajectory centers atop the image
        for index in xrange(1, len(self.path.trajectory) - 1):
            [cellX, cellY, rotation] = self.path.trajectory[index].position_desired
            stateH = int(round(originH + (cellY * horz_pixels_per_cell)))
            stateV = int(round(originV + (cellX * vert_pixels_per_cell)))
            point1 = (stateH - 2, stateV - 2)
            point2 = (stateH + 2, stateV + 2)
            cv.Rectangle(sdf_false_color, point1, point2, (0, 127, 0))
        # Draw the requested state
        # Draw the current center point
        centerH = int(round(originH + (current_position[1] * horz_pixels_per_cell)))
        centerV = int(round(originV + (current_position[0] * vert_pixels_per_cell)))
        cv.Circle(sdf_false_color, (centerH, centerV), 3, (0, 255, 255), thickness=-1)
        # Make the list of all points in the robot
        cx = 11
        cy = 4
        robot_voxels = []
        for xr in xrange(16):
            for yr in xrange(16):
                if (xr < 8) and (yr >= 8):
                    pass
                else:
                    rx = xr - cx
                    if rx >= 0.0:
                        rx -= 0.5
                    elif rx <= -0.0:
                        rx += 0.5
                    ry = yr - cy
                    if ry >= 0.0:
                        ry -= 0.5
                    elif ry <= -0.0:
                        ry += 0.5
                    robot_voxels.append([rx, ry])
        # Transform each of the points in the robot into environment (x,y) locations
        environment_cells = []
        for (rbx, rby) in robot_voxels:
            [ex, ey] = self.transform(current_position[0], current_position[1], current_position[2], rbx, rby)
            environment_cells.append([ex, ey])
        print "Robot voxels: ", robot_voxels
        print "Environment cells: ", environment_cells
        # Convert the environment cells to pixels
        pixel_points = []
        for (ex, ey) in environment_cells:
            eH = int(round(originH + (ey * horz_pixels_per_cell)))
            eV = int(round(originV + (ex * vert_pixels_per_cell)))
            pixel_points.append([eH, eV])
            #cv.Circle(sdf_false_color, (eH,eV), 3, (0,127,127))
        print "Pixel points: ", pixel_points
        ################################################################################################################
        # Compute the gradient for each point
        gradients = []
        for (eH, eV) in pixel_points:
            if (eH >= 1) and (eH < (current_sdf.cols - 1)):
                if (eV >= 1) and (eV < (current_sdf.rows - 1)):
                    gHorz = (self.query_sdf(current_sdf, eV, eH + 1) - self.query_sdf(current_sdf, eV, eH - 1)) * 0.5
                    gVert = (self.query_sdf(current_sdf, eV + 1, eH) - self.query_sdf(current_sdf, eV - 1, eH)) * 0.5
                    # Convert values from SDF into cells, not pixels
                    gHorz = gHorz / horz_pixels_per_cell
                    gVert = gVert / vert_pixels_per_cell
                    sdf_val = self.query_sdf(current_sdf, eV, eH)
                    if sdf_val >= self.sdf_threshold:
                        gHorz = 0.0
                        gVert = 0.0
                    else:
                        dist_to_threshold = abs(self.sdf_threshold - sdf_val)
                        gHorz = gHorz * self.gradient_multiplier * dist_to_threshold
                        gVert = gVert * self.gradient_multiplier * dist_to_threshold
                    gradients.append([gVert, gHorz])
                    # Draw the gradient
                    cv.Circle(sdf_false_color, (eH,eV), 1, (127,127,0))
                    cv.Line(sdf_false_color, (eH,eV), (int(round(eH + 5 * gHorz)), int(round(eV + 5 * gVert))), (100,100,100))
                else:
                    rospy.logwarn("eV " + str(eV) + " outside bounds")
                    gradients.append([0.0, 0.0])
            else:
                rospy.logwarn("eH " + str(eH) + " outside bounds")
                gradients.append([0.0, 0.0])
        # Combine the gradients together
        # Loop through to build the Jacobian (J) and end-effector gradient (Xdot) matrices
        # since Xdot = J * Qdot
        J_raw = []
        Xdot_raw = []
        for index in xrange(len(gradients)):
            gradient = gradients[index]
            Xdot_raw += gradient
            # Compute the Jacobian at the current state
            d = math.sqrt((robot_voxels[index][0] * vert_pixels_per_cell) ** 2 + (robot_voxels[index][1] * horz_pixels_per_cell) ** 2)
            d2 = math.sqrt((robot_voxels[index][0]) ** 2 + (robot_voxels[index][1]) ** 2)
            if (abs(gradient[0]) > 0.0 or abs(gradient[1]) > 0.0):
                pass #print "d value: ", d
            # Compute the current derivatives for rotation
            px = environment_cells[index][0] - current_position[0]
            py = environment_cells[index][1] - current_position[1]
            angle = math.atan2(py, px)
            pprimex = d2 * cos(0.1 + angle)
            pprimey = d2 * sin(0.1 + angle)
            deltapx = (pprimex - px) / 0.1
            deltapy = (pprimey - py) / 0.1
            #print "d2:", d2
            #print "angle:", angle
            #print "dpx:", deltapx
            #print "dpy:", deltapy
            J_raw.append([1.0, 0.0, deltapx])
            J_raw.append([0.0, 1.0, deltapy])
            #J_raw.append([1.0, 0.0, -d * sin(angle)])
            #J_raw.append([0.0, 1.0, d * cos(angle)])
            #J_raw.append([1.0, 0.0, d * -sin(1.0)]) # 2.0 inflates the rotation effect to decrease rotation gradient
            #J_raw.append([0.0, 1.0, d * cos(1.0)]) # same here 
        # Draw the last point
        #cv.Circle(sdf_false_color, (int(round(originH + (environment_cells[-1][1] * horz_pixels_per_cell))), int(round(originV + (environment_cells[-1][0] * vert_pixels_per_cell)))), 3, (255,255,255))
        J = numpy.array(J_raw)
        Xdot = numpy.array(Xdot_raw)
        #print "Xdot", Xdot
        #print "J", J
        # 'Invert' the Jacobian so we can do Qdot = J+ * Xdot
        Jpinv = numpy.linalg.pinv(J)
        #Jpinv = numpy.transpose(J)
        #print "J+", Jpinv
        # Compute Qdot
        Qdot = numpy.dot(Jpinv, Xdot)
        print "Current gradient is " + str(Qdot) + " (unbounded)"
        ################################################################################################################
        # Draw the complete (x,y) gradient
        cv.Line(sdf_false_color, (centerH,centerV), (int(round(centerH + (10 * Qdot[1]))), int(round(centerV + (10 * Qdot[0])))), (0,255,255))
        return [sdf_false_color, Qdot, originH, originV, horz_pixels_per_cell, vert_pixels_per_cell, robot_voxels]

if __name__ == '__main__':
    print bcolors.HEADER + bcolors.BOLD + "[SDF CLIENT] Starting the SDF client..." + bcolors.ENDC
    #Get the parameters from the server
    sdf_topic = rospy.get_param("~sdf_topic", "test_camera/sdf_raw")
    centers_topic = rospy.get_param("~centers_topic", "test_camera/centers")
    #Print the starting parameters to the screen
    print bcolors.HEADER + bcolors.BOLD + "[SDF CLIENT] Starting with SDF topic: /" + sdf_topic + bcolors.ENDC
    print bcolors.HEADER + bcolors.BOLD + "[SDF CLIENT] Starting with centers topic: /" + centers_topic + bcolors.ENDC
    sdf_threshold = 20.0
    gradient_multiplier = 5.0
    SDFClient(sdf_topic, centers_topic, sdf_threshold, gradient_multiplier)
