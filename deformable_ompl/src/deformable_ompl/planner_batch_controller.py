#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2015, Calder Phillips-Grafflin    #
#                                                   #
#   Batch frontend to the OMPL deformable object    #
#   motion planner. This node connects to n copies  #
#   of the motion planner node, and distributes     #
#   planning queries across the connected nodes.    #
#                                                   #
#####################################################

# System imports
import sys
import os
import string
from copy import *
import threading
import rospy
from deformable_ompl.msg import *
from deformable_ompl.srv import *


class PlannerConnection(object):

    def __init__(self, planner_node_name):
        self.planner_node_name = planner_node_name
        rospy.loginfo("Planner connection to node [" + self.planner_node_name + "]...")
        self.plan_path_service = rospy.ServiceProxy(self.planner_node_name + "/plan_path", PlanPath)
        self.plan_path_service.wait_for_service()
        self.compute_dvxl_cost_service = rospy.ServiceProxy(self.planner_node_name + "/compute_dvxl_cost", ComputeDVXLCost)
        self.compute_dvxl_cost_service.wait_for_service()
        self.resample_path_service = rospy.ServiceProxy(self.planner_node_name + "/resample_path", ResamplePath)
        self.resample_path_service.wait_for_service()
        self.simplify_path_service = rospy.ServiceProxy(self.planner_node_name + "/simplify_path", SimplifyPath)
        self.simplify_path_service.wait_for_service()
        self.compute_trial_setup_service = rospy.ServiceProxy(self.planner_node_name + "/compute_trial_setup", ComputeTrialSetup)
        self.compute_trial_setup_service.wait_for_service()
        rospy.loginfo("...connected to node [" + self.planner_node_name + "]")
        self.available = True

    def is_available(self):
        return self.available

    def plan_path(self, request):
        if self.available:
            self.available = False
            rospy.loginfo("Planner node [" + self.planner_node_name + "] handling PlanPath service")
            response = self.plan_path_service.call(request)
            self.available = True
            return response
        else:
            rospy.logerr("Planner node [" + self.planner_node_name + "] is already busy")
            return None

    def compute_dvxl_cost(self, request):
        if self.available:
            self.available = False
            rospy.loginfo("Planner node [" + self.planner_node_name + "] handling ComputeDVXLCost service")
            response = self.compute_dvxl_cost_service.call(request)
            self.available = True
            return response
        else:
            rospy.logerr("Planner node [" + self.planner_node_name + "] is already busy")
            return None

    def resample_path(self, request):
        if self.available:
            self.available = False
            rospy.loginfo("Planner node [" + self.planner_node_name + "] handling ResamplePath service")
            response = self.resample_path_service.call(request)
            self.available = True
            return response
        else:
            rospy.logerr("Planner node [" + self.planner_node_name + "] is already busy")
            return None

    def simplify_path(self, request):
        if self.available:
            self.available = False
            rospy.loginfo("Planner node [" + self.planner_node_name + "] handling SimplifyPath service")
            response = self.simplify_path_service.call(request)
            self.available = True
            return response
        else:
            rospy.logerr("Planner node [" + self.planner_node_name + "] is already busy")
            return None

    def compute_trial_setup(self, request):
        if self.available:
            self.available = False
            rospy.loginfo("Planner node [" + self.planner_node_name + "] handling ComputeTrialSetup service")
            response = self.compute_trial_setup_service.call(request)
            self.available = True
            return response
        else:
            rospy.logerr("Planner node [" + self.planner_node_name + "] is already busy")
            return None


class PlannerBatchFrontend(object):

    def __init__(self, planner_node_names):
        rospy.init_node("planner_batch_controller")
        # Make the state storage
        self.requests = 0
        self.plan_path_requests = []
        self.plan_path_responses = {}
        self.compute_dvxl_cost_requests = []
        self.compute_dvxl_cost_responses = {}
        self.resample_path_requests = []
        self.resample_path_responses = {}
        self.simplify_path_requests = []
        self.simplify_path_responses = {}
        self.compute_trial_setup_requests = []
        self.compute_trial_setup_responses = {}
        # Make the planner connections
        rospy.loginfo("Connecting to all planner nodes...")
        self.connections = []
        for planner_node_name in planner_node_names:
            new_connection = PlannerConnection(planner_node_name)
            self.connections.append(new_connection)
        self.available_planners = copy(self.connections)
        rospy.loginfo("...connected")
        # Make the service interfaces
        self.single_plan_path_service = rospy.Service("plan_path", PlanPath, self.single_plan_path_cb)
        self.single_compute_dvxl_cost_service = rospy.Service("compute_dvxl_cost", ComputeDVXLCost, self.single_compute_dvxl_cost_cb)
        self.single_resample_path_service = rospy.Service("resample_path", ResamplePath, self.single_resample_path_cb)
        self.single_simplify_path_service = rospy.Service("simplify_path", SimplifyPath, self.single_simplify_path_cb)
        self.single_compute_trial_setup_service = rospy.Service("compute_trial_setup", ComputeTrialSetup, self.single_compute_trial_setup_cb)
        self.batch_plan_path_service = rospy.Service("batch_plan_path", BatchPlanPath, self.batch_plan_path_cb)
        self.batch_compute_dvxl_cost_service = rospy.Service("batch_compute_dvxl_cost", BatchComputeDVXLCost, self.batch_compute_dvxl_cost_cb)
        self.batch_resample_path_service = rospy.Service("batch_resample_path", BatchResamplePath, self.batch_resample_path_cb)
        self.batch_simplify_path_service = rospy.Service("batch_simplify_path", BatchSimplifyPath, self.batch_simplify_path_cb)
        self.batch_compute_trial_setup_service = rospy.Service("batch_compute_trial_setup", BatchComputeTrialSetup, self.batch_compute_trial_setup_cb)
        rospy.loginfo("Planner batch controller setup complete...")

    def safe_get(self, container, index):
        if index < len(container):
            return container[index]
        else:
            return None

    def dispatch_plan_path(self, planner, (request_id, request)):
        thread = threading.Thread(target=self.plan_path_wrapper, args=(planner, request_id, request, self.handle_plan_path_response))
        thread.start()

    def handle_plan_path_response(self, (request_id, response)):
        self.plan_path_responses[request_id] = response

    def plan_path_wrapper(self, planner, request_id, request, response_callback):
        response = planner.plan_path(request)
        response_callback((request_id, response))
        self.available_planners.append(planner)

    def dispatch_resample_path(self, planner, (request_id, request)):
        thread = threading.Thread(target=self.resample_path_wrapper, args=(planner, request_id, request, self.handle_resample_path_response))
        thread.start()

    def handle_resample_path_response(self, (request_id, response)):
        self.resample_path_responses[request_id] = response

    def resample_path_wrapper(self, planner, request_id, request, response_callback):
        response = planner.resample_path(request)
        response_callback((request_id, response))
        self.available_planners.append(planner)

    def dispatch_simplify_path(self, planner, (request_id, request)):
        thread = threading.Thread(target=self.simplify_path_wrapper, args=(planner, request_id, request, self.handle_simplify_path_response))
        thread.start()

    def handle_simplify_path_response(self, (request_id, response)):
        self.simplify_path_responses[request_id] = response

    def simplify_path_wrapper(self, planner, request_id, request, response_callback):
        response = planner.simplify_path(request)
        response_callback((request_id, response))
        self.available_planners.append(planner)

    def dispatch_compute_dvxl_cost(self, planner, (request_id, request)):
        thread = threading.Thread(target=self.compute_dvxl_cost_wrapper, args=(planner, request_id, request, self.handle_compute_dvxl_cost_response))
        thread.start()

    def handle_compute_dvxl_cost_response(self, (request_id, response)):
        self.compute_dvxl_cost_responses[request_id] = response

    def compute_dvxl_cost_wrapper(self, planner, request_id, request, response_callback):
        response = planner.compute_dvxl_cost(request)
        response_callback((request_id, response))
        self.available_planners.append(planner)

    def dispatch_compute_trial_setup(self, planner, (request_id, request)):
        thread = threading.Thread(target=self.compute_trial_setup_wrapper, args=(planner, request_id, request, self.handle_compute_trial_setup_response))
        thread.start()

    def handle_compute_trial_setup_response(self, (request_id, response)):
        self.compute_trial_setup_responses[request_id] = response

    def compute_trial_setup_wrapper(self, planner, request_id, request, response_callback):
        response = planner.compute_trial_setup(request)
        response_callback((request_id, response))
        self.available_planners.append(planner)

    def loop(self):
        rate = rospy.Rate(rospy.get_param('~hz', 5))
        while not rospy.is_shutdown():
            # Service the request queues
            # Check to make sure there's work to do
            if len(self.plan_path_requests) > 0 or len(self.resample_path_requests) > 0 or len(self.simplify_path_requests) > 0 or len(self.compute_dvxl_cost_requests) > 0 or len(self.compute_trial_setup_requests) > 0:
                # First, check how many planners are available to do work
                available_planners = self.available_planners
                # If everyone is busy, do nothing
                if len(available_planners) == 0:
                    pass
                else:
                    # Get a task for each planner
                    while len(available_planners) > 0:
                        # Get the lowest request id from all the request queues
                        pp_req = self.safe_get(self.plan_path_requests, 0)
                        rp_req = self.safe_get(self.resample_path_requests, 0)
                        sp_req = self.safe_get(self.simplify_path_requests, 0)
                        cdc_req = self.safe_get(self.compute_dvxl_cost_requests, 0)
                        cts_req = self.safe_get(self.compute_trial_setup_requests, 0)
                        request_ids = []
                        if pp_req is not None:
                            request_ids.append(pp_req[0])
                        if rp_req is not None:
                            request_ids.append(rp_req[0])
                        if sp_req is not None:
                            request_ids.append(sp_req[0])
                        if cdc_req is not None:
                            request_ids.append(cdc_req[0])
                        if cts_req is not None:
                            request_ids.append(cts_req[0])
                        # Get the lowest request id
                        if len(request_ids) > 0:
                            best_request_id = min(request_ids)
                            # Dispatch the request
                            if pp_req is not None and pp_req[0] == best_request_id:
                                self.plan_path_requests.remove(pp_req)
                                current_planner = available_planners[0]
                                available_planners.remove(current_planner)
                                self.dispatch_plan_path(current_planner, pp_req)
                            elif rp_req is not None and rp_req[0] == best_request_id:
                                self.resample_path_requests.remove(rp_req)
                                current_planner = available_planners[0]
                                available_planners.remove(current_planner)
                                self.dispatch_resample_path(current_planner, rp_req)
                            elif sp_req is not None and sp_req[0] == best_request_id:
                                self.simplify_path_requests.remove(sp_req)
                                current_planner = available_planners[0]
                                available_planners.remove(current_planner)
                                self.dispatch_simplify_path(current_planner, sp_req)
                            elif cdc_req is not None and cdc_req[0] == best_request_id:
                                self.compute_dvxl_cost_requests.remove(cdc_req)
                                current_planner = available_planners[0]
                                available_planners.remove(current_planner)
                                self.dispatch_compute_dvxl_cost(current_planner, cdc_req)
                            elif cts_req is not None and cts_req[0] == best_request_id:
                                self.compute_trial_setup_requests.remove(cts_req)
                                current_planner = available_planners[0]
                                available_planners.remove(current_planner)
                                self.dispatch_compute_trial_setup(current_planner, cts_req)
                            else:
                                rospy.logerr("Should not be here")
                        else:
                            pass
            else:
                pass
            rate.sleep()
        rospy.loginfo("Shutting down...")

    def get_request_id(self):
        self.requests += 1
        return "req_" + str(self.requests)

    def enqueue_plan_path_request(self, request_id, request):
        self.plan_path_requests.append([request_id, request])

    def enqueue_compute_dvxl_cost_request(self, request_id, request):
        self.compute_dvxl_cost_requests.append([request_id, request])

    def enqueue_compute_trial_setup_request(self, request_id, request):
        self.compute_trial_setup_requests.append([request_id, request])

    def enqueue_resample_path_request(self, request_id, request):
        self.resample_path_requests.append([request_id, request])

    def enqueue_simplify_path_request(self, request_id, request):
        self.simplify_path_requests.append([request_id, request])

    def retrieve_plan_path_response(self, request_id):
        if request_id in self.plan_path_responses:
            response = self.plan_path_responses[request_id]
            del self.plan_path_responses[request_id]
            return response
        else:
            return None

    def retrieve_compute_dvxl_cost_response(self, request_id):
        if request_id in self.compute_dvxl_cost_responses:
            response = self.compute_dvxl_cost_responses[request_id]
            del self.compute_dvxl_cost_responses[request_id]
            return response
        else:
            return None

    def retrieve_compute_trial_setup_response(self, request_id):
        if request_id in self.compute_trial_setup_responses:
            response = self.compute_trial_setup_responses[request_id]
            del self.compute_trial_setup_responses[request_id]
            return response
        else:
            return None

    def retrieve_resample_path_response(self, request_id):
        if request_id in self.resample_path_responses:
            response = self.resample_path_responses[request_id]
            del self.resample_path_responses[request_id]
            return response
        else:
            return None

    def retrieve_simplify_path_response(self, request_id):
        if request_id in self.simplify_path_responses:
            response = self.simplify_path_responses[request_id]
            del self.simplify_path_responses[request_id]
            return response
        else:
            return None

    def batch_plan_path_cb(self, request):
        # Enqueue the requests
        request_ids = []
        for pp_request in request.queries:
            request_id = self.get_request_id()
            request_ids.append(request_id)
            self.enqueue_plan_path_request(request_id, pp_request)
        # Wait for them all to complete
        responses = {}
        for request_id in request_ids:
            responses[request_id] = self.retrieve_plan_path_response(request_id)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        while None in responses.values():
            rate.sleep()
            for key, val in responses.iteritems():
                if val is None:
                    responses[key] = self.retrieve_plan_path_response(key)
        # Extract in the proper order
        results = []
        for request_id in request_ids:
            results.append(responses[request_id].result)
        # Return
        response = BatchPlanPathResponse()
        response.results = results
        return response

    def batch_resample_path_cb(self, request):
        # Enqueue the requests
        request_ids = []
        for rp_request in request.queries:
            request_id = self.get_request_id()
            request_ids.append(request_id)
            self.enqueue_resample_path_request(request_id, rp_request)
        # Wait for them all to complete
        responses = {}
        for request_id in request_ids:
            responses[request_id] = self.retrieve_resample_path_response(request_id)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        while None in responses.values():
            rate.sleep()
            for key, val in responses.iteritems():
                if val is None:
                    responses[key] = self.retrieve_resample_path_response(key)
        # Extract in the proper order
        results = []
        for request_id in request_ids:
            results.append(responses[request_id].result)
        # Return
        response = BatchResamplePathResponse()
        response.results = results
        return response

    def batch_simplify_path_cb(self, request):
        # Enqueue the requests
        request_ids = []
        for sp_request in request.queries:
            request_id = self.get_request_id()
            request_ids.append(request_id)
            self.enqueue_simplify_path_request(request_id, sp_request)
        # Wait for them all to complete
        responses = {}
        for request_id in request_ids:
            responses[request_id] = self.retrieve_simplify_path_response(request_id)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        while None in responses.values():
            rate.sleep()
            for key, val in responses.iteritems():
                if val is None:
                    responses[key] = self.retrieve_simplify_path_response(key)
        # Extract in the proper order
        results = []
        for request_id in request_ids:
            results.append(responses[request_id].result)
        # Return
        response = BatchSimplifyPathResponse()
        response.results = results
        return response

    def batch_compute_dvxl_cost_cb(self, request):
        # Enqueue the requests
        request_ids = []
        for cdc_request in request.queries:
            request_id = self.get_request_id()
            request_ids.append(request_id)
            self.enqueue_compute_dvxl_cost_request(request_id, cdc_request)
        # Wait for them all to complete
        responses = {}
        for request_id in request_ids:
            responses[request_id] = self.retrieve_compute_dvxl_cost_response(request_id)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        while None in responses.values():
            rate.sleep()
            for key, val in responses.iteritems():
                if val is None:
                    responses[key] = self.retrieve_compute_dvxl_cost_response(key)
        # Extract in the proper order
        results = []
        for request_id in request_ids:
            results.append(responses[request_id].result)
        # Return
        response = BatchComputeDVXLCostResponse()
        response.results = results
        return response

    def batch_compute_trial_setup_cb(self, request):
        # Enqueue the requests
        request_ids = []
        for cts_request in request.queries:
            request_id = self.get_request_id()
            request_ids.append(request_id)
            self.enqueue_compute_trial_setup_request(request_id, cts_request)
        # Wait for them all to complete
        responses = {}
        for request_id in request_ids:
            responses[request_id] = self.retrieve_compute_trial_setup_response(request_id)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        while None in responses.values():
            rate.sleep()
            for key, val in responses.iteritems():
                if val is None:
                    responses[key] = self.retrieve_compute_trial_setup_response(key)
        # Extract in the proper order
        results = []
        for request_id in request_ids:
            results.append(responses[request_id].result)
        # Return
        response = BatchComputeTrialSetupResponse()
        response.results = results
        return response

    def single_plan_path_cb(self, request):
        request_id = self.get_request_id()
        self.enqueue_plan_path_request(request_id, request)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        response = self.retrieve_plan_path_response(request_id)
        while response is None:
            rate.sleep()
            response = self.retrieve_plan_path_response(request_id)
        return response

    def single_resample_path_cb(self, request):
        request_id = self.get_request_id()
        self.enqueue_resample_path_request(request_id, request)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        response = self.retrieve_resample_path_response(request_id)
        while response is None:
            rate.sleep()
            response = self.retrieve_resample_path_response(request_id)
        return response

    def single_simplify_path_cb(self, request):
        request_id = self.get_request_id()
        self.enqueue_simplify_path_request(request_id, request)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        response = self.retrieve_simplify_path_response(request_id)
        while response is None:
            rate.sleep()
            response = self.retrieve_simplify_path_response(request_id)
        return response

    def single_compute_dvxl_cost_cb(self, request):
        request_id = self.get_request_id()
        self.enqueue_compute_dvxl_cost_request(request_id, request)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        response = self.retrieve_compute_dvxl_cost_response(request_id)
        while response is None:
            rate.sleep()
            response = self.retrieve_compute_dvxl_cost_response(request_id)
        return response

    def single_compute_trial_setup_cb(self, request):
        request_id = self.get_request_id()
        self.enqueue_compute_trial_setup_request(request_id, request)
        rate = rospy.Rate(rospy.get_param('~hz', 0.5))
        response = self.retrieve_compute_trial_setup_response(request_id)
        while response is None:
            rate.sleep()
            response = self.retrieve_compute_trial_setup_response(request_id)
        return response

if __name__ == '__main__':
    if len(sys.argv) > 1:
        possible_planner_node_names = sys.argv[1:]
        planner_node_names = []
        for possible_planner_node_name in possible_planner_node_names:
            if ":=" not in possible_planner_node_name:
                planner_node_names.append(possible_planner_node_name)
        PBC = PlannerBatchFrontend(planner_node_names)
        PBC.loop()
    else:
        print("Too few arguments provided")




