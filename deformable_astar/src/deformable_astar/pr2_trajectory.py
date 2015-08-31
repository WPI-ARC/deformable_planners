#!/usr/bin/python

import rospy
import subprocess
import actionlib
import time
from actionlib import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from simple_robot_kinematics.pr2_simple_kinematics import *
from tf.transformations import *
from transformation_helper import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from deformable_astar.msg import *
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

class PR2TrajectoryClient:

    def __init__(self, path, start, goal, sim_only, gen_all_paths, vision, p_weight=0.5):
        self.path = path
        rospy.init_node("planner_client")
        self.IK_client = SimpleInverseKinematics()
        self.planner_client = rospy.ServiceProxy("/astar_deform/PlannerQuery", DeformPlanner)
        print "Planner client loaded"
        self.right_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.right_arm_client.wait_for_server()
        print "Arm client loaded"
        self.right_gripper_client = actionlib.SimpleActionClient("r_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.right_gripper_client.wait_for_server()
        print "Gripper client loaded"
        self.camera_client = rospy.ServiceProxy("camera_client/CameraTrigger", CameraTrigger)
        print "Camera client loaded"
        self.close_gripper_goal = Pr2GripperCommandGoal()
        self.close_gripper_goal.command.position = 0.00
        self.close_gripper_goal.command.max_effort = -1.0
        print "Calling the A* deformable environment planner"
        time.sleep(1)
        #Call the planner
        planned_paths = self.run_planner(start, goal, sim_only, gen_all_paths, p_weight)
        if (sim_only):
            for [planned_path, p_value] in planned_paths:
                print "Got solution with " + str(len(planned_path)) + " steps, length " + str(planned_path[-1].distance) + " and deformation " + str(planned_path[-1].deformation) + " using pareto weight " + str(p_value)
            print bcolors.QUESTION + bcolors.BOLD + "Exiting..." + bcolors.ENDC
        elif (vision):
            for [planned_path, p_value] in planned_paths:
                print "Executing [w/ vision] solution with " + str(len(planned_path)) + " steps, length " + str(planned_path[-1].distance) + " and deformation " + str(planned_path[-1].deformation) + " using pareto weight " + str(p_value)
                #Convert to a trajectory
                [forward, backward] = self.convert_fully(planned_path)
                #Print it out for safety
                print bcolors.WARNING + "Planner produced the following trajectory:" + bcolors.ENDC
                #for state in left_trajrequest.points:
                #    print state
                #Run the planned trajectory
                print bcolors.CYAN + "Computing average noise..." + bcolors.ENDC
                noise_floor = self.find_noise_floor(10)
                print bcolors.OKBLUE + bcolors.BOLD + "Noise floor for this session is " + str(noise_floor) + bcolors.ENDC
                raw_input(bcolors.WARNING + "Press ENTER to continue..." + bcolors.ENDC)
                #Close the gripper
                self.left_gripper_client.send_goal(self.close_gripper_goal)
                time.sleep(2.0)
                print bcolors.CYAN + "Running the planned trajectory" + bcolors.ENDC
                [total_deformationF, deformationsF] = self.run_trajectory(forward, noise_floor)
                print bcolors.OKGREEN + "Trajectory execution complete, reversing..." + bcolors.ENDC
                [total_deformationB, deformationsB] = self.run_trajectory(backward, noise_floor)
                print bcolors.OKGREEN + "Reverse Trajectory execution complete" + bcolors.ENDC
                self.analyze_results(total_deformationF, deformationsF, total_deformationB, deformationsB, planned_path)
                #Cancel the gripper command
                self.left_gripper_client.cancel_goal()
                print bcolors.QUESTION + bcolors.BOLD + "Exiting..." + bcolors.ENDC
        else:
            for [planned_path, p_value] in planned_paths:
                print "Executing [w/o vision] solution with " + str(len(planned_path)) + " steps, length " + str(planned_path[-1].distance) + " and deformation " + str(planned_path[-1].deformation) + " using pareto weight " + str(p_value)
                #Convert to a trajectory
                [forward, backward] = self.convert_fully(planned_path)
                #Print it out for safety
                print bcolors.WARNING + "Planner produced the following trajectory:" + bcolors.ENDC
                #for state in left_trajrequest.points:
                #    print state
                raw_input(bcolors.WARNING + "Press ENTER to continue..." + bcolors.ENDC)
                #Run the planned trajectory
                print bcolors.CYAN + "Running the planned trajectory" + bcolors.ENDC
                left_goal = JointTrajectoryGoal()
                left_goal.trajectory = forward
                self.left_arm_client.send_goal(left_goal)
                self.left_arm_client.wait_for_result()
                print bcolors.OKGREEN + "Trajectory execution complete" + bcolors.ENDC
                back_goal = JointTrajectoryGoal()
                back_goal.trajectory = backward
                self.left_arm_client.send_goal(back_goal)
                self.left_arm_client.wait_for_result()
                print bcolors.OKGREEN + "Reverse Trajectory execution complete" + bcolors.ENDC
                print bcolors.QUESTION + bcolors.BOLD + "Exiting..." + bcolors.ENDC

    def analyze_results(self, forward_total, forward_all, backward_total, backward_all, planned_path):
        print "Forward pass produced total deformation : " + str(forward_total)
        print "Backward pass produced total deformation : " + str(backward_total)
        print "Difference : " + str(abs(forward_total - backward_total))
        print "Per-state deformation:"
        assert(len(forward_all) == len(backward_all) == len(planned_path))
        outfile = open(self.path + "/deformation.csv", "w")
        filestring = "Computed,Computed sum,Measured forward,Measured backward,Measured forward sum,Measured backward sum\n"
        total_f = 0.0
        total_b = 0.0
        last_computed = 0.0
        for state in range(len(planned_path)):
            computed_sum = planned_path[state].deformation
            measured_f = forward_all[state]
            measured_b = backward_all[len(backward_all) - 1 - state]
            total_f += measured_f
            total_b += measured_b
            computed = computed_sum - last_computed
            last_computed = computed_sum
            if ((computed == 0.0 and measured_f == 0.0 and measured_b == 0.0) or (computed != 0.0)):
                print "Computed: " + str(computed) + " | Computed [sum]: " + str(computed_sum) + " | Measured forward: " + str(measured_f) + " | Measured forward [sum]: " + str(total_f) + " | Measured backward: " + str(measured_b) + " | Measured backward [sum]: " + str(total_b) + " | F/B Difference: " + str(abs(measured_f - measured_b))
            else:
                print bcolors.WARNING + "Computed: " + str(computed) + " | Computed [sum]: " + str(computed_sum) + " | Measured forward: " + str(measured_f) + " | Measured forward [sum]: " + str(total_f) + " | Measured backward: " + str(measured_b) + " | Measured backward [sum]: " + str(total_b) + " | F/B Difference: " + str(abs(measured_f - measured_b)) + bcolors.ENDC
            filestring += str(computed) + "," + str(computed_sum) + "," + str(measured_f) + "," + str(measured_b) + "," + str(total_f) + "," + str(total_b) + "\n"
        outfile.write(filestring)
        outfile.close()

    def run_trajectory(self, trajectory, noise_floor):
        deformations = []
        total_deformation = 0.0
        for state in trajectory.points:
            state.time_from_start = rospy.Duration(0.5)
            temp_traj = JointTrajectory()
            temp_traj.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
            temp_traj.points = [state]
            temp_goal = JointTrajectoryGoal()
            temp_goal.trajectory = temp_traj
            self.left_arm_client.send_goal(temp_goal)
            self.left_arm_client.wait_for_result()
            measured = self.compute_deform_with_camera(0, noise_floor)
            deformations.append(measured)
            total_deformation += measured
            print bcolors.CYAN + "Camera measured deformation of " + str(measured) + " for a current total of " + str(total_deformation)
        return [total_deformation, deformations]

    def find_noise_floor(self, iterations):
        sum_def = 0
        i = 0
        measurements = []
        while (i < iterations):
            measured = self.camera_client.call(1)
            if (measured.deformation != -1.0):
                sum_def += measured.deformation
                i += 1
                measurements.append(measured.deformation)
            rospy.sleep(0.1)
        avg_def = sum_def / float(iterations)
        min_def = min(measurements)
        max_def = max(measurements)
        return [avg_def, min_def, max_def]

    def compute_deform_with_camera(self, reset, noise_floor_data):
        noise_floor = noise_floor_data[0]
        min_floor = noise_floor_data[1]
        max_floor = noise_floor_data[2]
        noise_range = max(abs(abs(noise_floor) - abs(min_floor)), abs(abs(noise_floor) - abs(max_floor)))
        sum_def = 0
        iterations = 1
        i = 0
        while (i < iterations):
            measured = self.camera_client.call(0)
            if (measured.deformation != -1.0):
                sum_def += (measured.deformation - noise_floor)
                i += 1
            rospy.sleep(0.1)
        avg_def = sum_def / float(iterations)
        if (avg_def > -300.0):
            return 0.0
        else:
            return avg_def + noise_range

    def run_planner(self, start, goal, sim_only, gen_all_paths, p_weight):
        generated_paths = {}
        returned_paths = []
        if (sim_only or gen_all_paths):
            previous_key = None
            for i in range(101):
                p_weight = float(i) / 100.0
                solution = self.call_planner(start, goal, p_weight)
                solution_key = str(len(solution)) + "|" + str(solution[-1].distance) + "|" + str(solution[-1].deformation)
                if (previous_key is not None):
                    if (solution_key != previous_key):
                        print bcolors.CYAN + "Re-searching the last increment at finer resolution" + bcolors.ENDC
                        tmp_p_weight = p_weight - 0.01
                        while (tmp_p_weight < p_weight):
                            int_solution = self.call_planner(start, goal, p_weight)
                            int_solution_key = str(len(int_solution)) + "|" + str(int_solution[-1].distance) + "|" + str(int_solution[-1].deformation)
                            if (int_solution_key != solution_key):
                                generated_paths[int_solution_key] = [int_solution, tmp_p_weight]
                            tmp_p_weight += 0.001
                generated_paths[solution_key] = [solution, p_weight]
                previous_key = solution_key
            for key in generated_paths.keys():
                returned_paths.append(generated_paths[key])
            return returned_paths
        else:
            return [[self.call_planner(start, goal, p_weight),p_weight]]
    
    def call_planner(self, start, goal, pareto_ctrl):
        #Call the deformable planner
        srv_req = DeformPlanner._request_class()
        srv_req.start = start
        srv_req.goal = goal
        srv_req.scale = 0.005
        srv_req.ctrl = 3
        srv_req.pareto_ctrl = pareto_ctrl
        srv_res = self.planner_client.call(srv_req)
        return srv_res.solution

    def convert_planner_state_to_pose(self, planner_state):
        #convert the 6-dof planner state to XYZ + Quaternion pose
        planner_pose = Pose()
        planner_pose.position.x = planner_state.xt + 0.25
        planner_pose.position.y = planner_state.yt
        planner_pose.position.z = planner_state.zt + 0.9
        Xq = quaternion_about_axis(planner_state.xr, (1,0,0))
        Yq = quaternion_about_axis(planner_state.yr, (0,1,0))
        Zq = quaternion_about_axis(planner_state.zr, (0,0,1))
        tq = ComposeQuaternions(Xq, Yq)
        Pq = ComposeQuaternions(tq, Zq)
        Bq = quaternion_about_axis(-math.pi/2.0, (0,1,0))
        Fq = ComposeQuaternions(Bq, Pq)
        planner_pose.orientation.x = Fq[0]
        planner_pose.orientation.y = Fq[1]
        planner_pose.orientation.z = Fq[2]
        planner_pose.orientation.w = Fq[3]
        planner_stamped = PoseStamped()
        planner_stamped.pose = planner_pose
        planner_stamped.header.frame_id = "base_link"
        return planner_stamped

    def convert_fully(self, path):
        print bcolors.CYAN + "Attempting to convert path to trajectory " + bcolors.ENDC
        running = True
        best_conversion = None
        iteration = 1
        max_iters = 5
        while (running and iteration < max_iters):
            [best_conversion, success, failure] = self.convert_path_to_trajectory(path)
            if (failure == 0):
                print bcolors.OKBLUE + "Complete conversion successful" + bcolors.ENDC
                running = False
            else:
                print bcolors.WARNING + "Complete conversion failed, repeating" + bcolors.ENDC
            iteration += 1
        back = self.reverse_trajectory(best_conversion)
        return [best_conversion, back]

    def reverse_trajectory(self, traj):
        reverse_traj = JointTrajectory()
        reverse_traj.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        reverse_traj.points = []
        time = 1.0
        for state in reversed(traj.points):
            new_point = JointTrajectoryPoint()
            new_point.positions = state.positions
            new_point.velocities = state.velocities
            new_point.time_from_start = rospy.Duration(time)
            reverse_traj.points.append(new_point)
            time += 0.5
        return reverse_traj

    def convert_path_to_trajectory(self, path):
        #convert the planned path to a pose path
        pose_path = []
        for point in path:
            pose_point = self.convert_planner_state_to_pose(point)
            pose_path.append(pose_point)
        #convert the pose path to a joint state path
        joint_path = []
        success = 0
        failure = 0
        for point in pose_path:
            #print bcolors.CYAN + "Attempting to convert:\n" + str(point) + bcolors.ENDC
            joint_state = self.IK_client.ComputeRightArmIK(point)
            if (joint_state is not None):
                #print bcolors.OKBLUE + "IK CALL RETURNED SUCCESS" + bcolors.ENDC
                joint_path.append(joint_state)
                success += 1
            else:
                #print bcolors.FAIL + bcolors.BOLD + "IK CALL RETURNED IMPOSSIBLE POSE" + bcolors.ENDC
                failure += 1
        print bcolors.WARNING + bcolors.BOLD + "IK CALLS - Successful: " + str(success) + " Failure: " + str(failure) + bcolors.ENDC
        #convert joint state path to joint trajectory
        left_trajrequest = JointTrajectory()
        left_trajrequest.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "l_wrist_roll_joint"]
        left_trajrequest.points = []
        time = 1.0
        for state in joint_path:
            new_point = JointTrajectoryPoint()
            new_point.positions = state
            new_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            new_point.time_from_start = rospy.Duration(time)
            left_trajrequest.points.append(new_point)
            time += 0.5
        return [left_trajrequest, success, failure]

if __name__ == '__main__':
    path = subprocess.check_output("rospack find astar_deform", shell=True)
    path = path.strip("\n") + "/data"
    #Set the start & goal pose
    #start
    sx = 71.0
    sy = 18.0
    sz = 2.0
    srx = 0.0
    sry = 0.0
    srz = 0.0
    #goal
    gx = 71.0
    gy = 96.0
    gz = 2.0
    grx = 0.0
    gry = 0.0
    grz = 0.0
    start = [sx,sy,sz,srx,sry,srz]
    goal = [gx,gy,gz,grz,gry,grz]
    print bcolors.HEADER + bcolors.BOLD + "[PR2 Trajectory Client] Starting the planner-trajectory client..." + bcolors.ENDC
    #Print the starting parameters to the screen
    print bcolors.HEADER + bcolors.BOLD + "[PR2 Trajectory Client] Start: " + str(start) + bcolors.ENDC
    print bcolors.HEADER + bcolors.BOLD + "[PR2 Trajectory Client] Goal: " + str(goal) + bcolors.ENDC
    #PR2TrajectoryClient(path, start, goal, False, False, True, 0.01)
    PR2TrajectoryClient(path, start, goal, False, False, False, 0.01)
    PR2TrajectoryClient(path, start, goal, False, False, False, 0.05)
    PR2TrajectoryClient(path, start, goal, False, False, False, 0.5)
