#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   Python/NUMPY implementation of LWO:             #
#   "Light-Weight Optimizer"                        #
#                                                   #
#####################################################

import math
import numpy
from copy import deepcopy


class LightWeightOptimizer(object):

    def __init__(self, max_optimization_iterations, max_optimization_step, convergence_total_delta_threshold,convergence_max_delta_threshold):
        self.max_optimization_iterations = max_optimization_iterations
        self.max_optimization_step_size = max_optimization_step
        self.convergence_total_delta_threshold = convergence_total_delta_threshold
        self.convergence_max_delta_threshold = convergence_max_delta_threshold

    def weighted_euclidean_distance(self, state1, state2, weights=None, dimensions=None):
        if weights is None:
            assert(len(state1) == len(state2))
            if dimensions is None:
                total = 0.0
                for index in range(len(state1)):
                    temp = (state1[index] - state2[index]) ** 2
                    total += temp
                return math.sqrt(total)
            else:
                assert(dimensions <= len(state1))
                total = 0.0
                for index in range(dimensions):
                    temp = (state1[index] - state2[index]) ** 2
                    total += temp
                return math.sqrt(total)
        else:
            assert(len(state1) == len(state2) == len(weights))
            if dimensions is None:
                total = 0.0
                for index in range(len(state1)):
                    temp = ((state1[index] - state2[index]) ** 2) * weights[index]
                    total += temp
                return math.sqrt(total)
            else:
                assert(dimensions <= len(state1))
                total = 0.0
                for index in range(dimensions):
                    temp = ((state1[index] - state2[index]) ** 2) * weights[index]
                    total += temp
                return math.sqrt(total)

    def interpolate_between_points(self, point1, point2, percent):
        interpolated = []
        assert(len(point1) == len(point2))
        points = zip(point1, point2)
        for p1i, p2i in points:
            interpolated_value = p1i + ((p2i - p1i) * percent)
            interpolated.append(interpolated_value)
        return numpy.array(interpolated)

    def uniformly_resample_path(self, path, num_samples=None):
        #   Uniformly resample the provided path, either with the same number of samples, or to a provided number
        if num_samples is None:
            num_samples = len(path)
        # Compute the current length of the path and the elapsed distance for each state
        total_distance = 0.0
        elapsed_distances = [0.0]
        for index in xrange(len(path) - 1):
            start_point = path[index]
            end_point = path[index + 1]
            distance = self.weighted_euclidean_distance(start_point, end_point)
            total_distance += distance
            elapsed_distances.append(total_distance)
        # Compute the target spacing between resampled points
        target_spacing = total_distance / float(num_samples - 1)
        # In addition to
        # We start the resampled path with the same initial state as the provided path
        resampled_path = [deepcopy(path[0])]
        # Step through the path, interpolating the intermediate points
        num_intermediate_samples = num_samples - 2
        current_sample_index = 1
        current_path_index = 1
        # Run until we've produced a full set of intermediate samples
        while current_sample_index < num_intermediate_samples:
            # Get how far the *current* state in the provided path is from the start
            current_elapsed_distance = elapsed_distances[current_path_index]
            # Get how far we *want* to be from the start
            target_elapsed_distance = target_spacing * float(current_sample_index)
            # Check if we're in the right window
            if target_elapsed_distance < current_elapsed_distance:
                # In the case where the *current* provided state is ahead of where we want to be,
                # we interpolate between the *previous* and *current* provided states
                # First, we need to compute what percent to
                previous_provided_state = path[current_path_index - 1]
                current_provided_state = path[current_path_index]
                previous_elapsed_distance = elapsed_distances[current_path_index - 1]
                current_window_distance = current_elapsed_distance - previous_elapsed_distance
                overlap_distance = target_elapsed_distance - previous_elapsed_distance
                percent_overlap = overlap_distance / current_window_distance
                # Interpolate between them
                interpolated_state = self.interpolate_between_points(previous_provided_state, current_provided_state, percent_overlap)
                # Store the interpolated point
                resampled_path.append(interpolated_state)
                # Update the control variables
                current_sample_index += 1
            elif target_elapsed_distance > current_elapsed_distance:
                # In the case where the *current* provided state is behind where we want to be,
                # we need to advance to the next window of the provided path
                current_path_index += 1
            else:
                # If we're exactly where we want to be, just copy the current state
                resampled_path.append(deepcopy(path[current_path_index]))
                current_sample_index += 1
        # Finish by adding the final state from the provided path
        resampled_path.append(deepcopy(path[-1]))
        return [resampled_path, correspondences]

    def compute_gradient_rejection(self, previous_point, current_point, next_point, raw_gradient):
        # Compute the current step vector (prev -> current)
        current_step_vector = numpy.subtract(current_point, previous_point)
        ##########
        # We don't use this option
        # # Compute the next step vector (current -> next)
        # next_step_vector = numpy.subtract(next_point, current_point)
        ##########
        # Pick which step vector we use for gradient rejection (GRC uses the current_step_vector)
        path_vector = current_step_vector
        # Compute the projection of the gradient vector onto the path vector
        projected_gradient = numpy.dot(path_vector, numpy.dot(raw_gradient, path_vector) / numpy.dot(path_vector, path_vector))
        # Compute the rejection of the gradient vector onto the path vector
        rejected_gradient = numpy.subtract(raw_gradient, projected_gradient)
        # Return
        return rejected_gradient

    def bound_optimization(self, raw_optimized_state, (preceding_reference_state_index, following_reference_state_index), reference_path, optimization_bound):
        # Ensure that the entire vector from base state to raw optimized state is within
        # the bounding volume of the reference path, and project it to the surface of the
        # bounding volume if it isn't
        # Set the preceding and following waypoints
        preceding_reference_waypoint = reference_path[preceding_reference_state_index]
        following_reference_waypoint = reference_path[following_reference_state_index]
        # Check if either of the waypoints is the first or last state of the reference path
        # If so, we constrain the optimized state so that it falls into a hypersphere around state 2 or n-1
        # with radius set by the optimization bound
        if preceding_reference_state_index == 0:
            # Get the following_reference_state->raw_optimized vector
            ref_to_optimized_vector = numpy.subtract(raw_optimized_state, following_reference_waypoint)
            # Compute the magnitude
            ref_to_optimized_vector_mag = numpy.linalg.norm(ref_to_optimized_vector)
            # Check if it's within bounds
            if abs(ref_to_optimized_vector_mag) <= abs(optimization_bound):
                # If so, return unmodified
                return raw_optimized_state
            else:
                # If not, constrain
                constrain_ratio = abs(optimization_bound) / abs(ref_to_optimized_vector_mag)
                # Make the constrained following_reference_state->optimized vector
                constrained_ref_to_optimized_vector = numpy.dot(ref_to_optimized_vector, constrain_ratio)
                # Make the constrained optimized state
                constrained_optimized_state = numpy.add(following_reference_waypoint, constrained_ref_to_optimized_vector)
                # Return the constrained state
                return constrained_optimized_state
        elif following_reference_state_index == (len(reference_path) - 1):
            # Get the preceding_reference_state->raw_optimized vector
            ref_to_optimized_vector = numpy.subtract(raw_optimized_state, preceding_reference_waypoint)
            # Compute the magnitude
            ref_to_optimized_vector_mag = numpy.linalg.norm(ref_to_optimized_vector)
            # Check if it's within bounds
            if abs(ref_to_optimized_vector_mag) <= abs(optimization_bound):
                # If so, return unmodified
                return raw_optimized_state
            else:
                # If not, constrain
                constrain_ratio = abs(optimization_bound) / abs(ref_to_optimized_vector_mag)
                # Make the constrained following_reference_state->optimized vector
                constrained_ref_to_optimized_vector = numpy.dot(ref_to_optimized_vector, constrain_ratio)
                # Make the constrained optimized state
                constrained_optimized_state = numpy.add(preceding_reference_waypoint, constrained_ref_to_optimized_vector)
                # Return the constrained state
                return constrained_optimized_state
        # If the point is between two intermediate waypoints, we compute two dot products:
        # dot1 = dot(preceding->optimized, preceding->following)
        # dot2 = dot(following->optimized, following->preceding)
        # From the dot products, we can tell if we need to check against the cylinder
        # between the points, or if we need to check against one of the point's hyperspheres
        preceding_to_optimized_vector = numpy.subtract(raw_optimized_state, preceding_reference_waypoint)
        preceding_to_following_vector = numpy.subtract(following_reference_waypoint, preceding_reference_waypoint)
        following_to_optimized_vector = numpy.subtract(raw_optimized_state, following_reference_waypoint)
        following_to_preceding_vector = numpy.subtract(preceding_reference_waypoint, following_reference_waypoint)
        dot1 = numpy.dot(preceding_to_optimized_vector, preceding_to_following_vector)
        dot2 = numpy.dot(following_to_optimized_vector, following_to_preceding_vector)
        # If either dot product is negative, we constrain to the relevant hypersphere
        if dot1 <= 0.0:
            # Constrain to the preceding waypoint's hypersphere
            # Get the preceding_reference_state->raw_optimized vector
            ref_to_optimized_vector = numpy.subtract(raw_optimized_state, preceding_reference_waypoint)
            # Compute the magnitude
            ref_to_optimized_vector_mag = numpy.linalg.norm(ref_to_optimized_vector)
            # Check if it's within bounds
            if abs(ref_to_optimized_vector_mag) <= abs(optimization_bound):
                # If so, return unmodified
                return raw_optimized_state
            else:
                # If not, constrain
                constrain_ratio = abs(optimization_bound) / abs(ref_to_optimized_vector_mag)
                # Make the constrained following_reference_state->optimized vector
                constrained_ref_to_optimized_vector = numpy.dot(ref_to_optimized_vector, constrain_ratio)
                # Make the constrained optimized state
                constrained_optimized_state = numpy.add(preceding_reference_waypoint, constrained_ref_to_optimized_vector)
                # Return the constrained state
                return constrained_optimized_state
        elif dot2 <= 0.0:
            # Constrain to the following waypoint's hypersphere
            # Get the following_reference_state->raw_optimized vector
            ref_to_optimized_vector = numpy.subtract(raw_optimized_state, following_reference_waypoint)
            # Compute the magnitude
            ref_to_optimized_vector_mag = numpy.linalg.norm(ref_to_optimized_vector)
            # Check if it's within bounds
            if abs(ref_to_optimized_vector_mag) <= abs(optimization_bound):
                # If so, return unmodified
                return raw_optimized_state
            else:
                # If not, constrain
                constrain_ratio = abs(optimization_bound) / abs(ref_to_optimized_vector_mag)
                # Make the constrained following_reference_state->optimized vector
                constrained_ref_to_optimized_vector = numpy.dot(ref_to_optimized_vector, constrain_ratio)
                # Make the constrained optimized state
                constrained_optimized_state = numpy.add(following_reference_waypoint, constrained_ref_to_optimized_vector)
                # Return the constrained state
                return constrained_optimized_state
        # If the optimized point is between the two waypoints, we make sure that it falls within the
        # hypercylinder defined by the two waypoints and the optimization bound
        # We project and reject the waypoint->optimized vector onto the waypoint->waypoint vector
        # If the magnitude of the rejection exceeds the optimization bound, we scale it down to match
        # and then combine the projection + scaled rejection to form the constrained waypoint->optimized
        # vector, which we use to produce the constrained optimized state.
        # Get the preceding waypoint->following waypoint vector
        preceding_to_following_vector = numpy.subtract(following_reference_waypoint, preceding_reference_waypoint)
        # Get the preceding waypoint->optimized vector
        preceding_to_optimized_vector = numpy.subtract(raw_optimized_state, preceding_reference_waypoint)
        # Compute the projection of preceding_to_optimized_vector onto preceding_to_following_vector
        projected_vector = numpy.dot(preceding_to_following_vector, numpy.dot(preceding_to_optimized_vector, preceding_to_following_vector) / numpy.dot(preceding_to_following_vector, preceding_to_following_vector))
        # Compute the rejection of preceding_to_optimized_vector onto preceding_to_following_vector
        rejected_vector = numpy.subtract(preceding_to_optimized_vector, projected_vector)
        # Compute the magnitude of the rejected vector
        rejected_vector_mag = numpy.linalg.norm(rejected_vector)
        # Check if it's within bounds
        if abs(rejected_vector_mag) <= abs(optimization_bound):
            # If so, return unmodified
            return raw_optimized_state
        else:
            # If not, constrain
            constrain_ratio = abs(optimization_bound) / abs(rejected_vector_mag)
            # Make the constrained rejection vector
            constrained_rejection_vector = numpy.dot(rejected_vector, constrain_ratio)
            # Combine the projection vector with the constrained rejection vector to
            # make the new preceding waypoint->constrained optimized vector
            preceding_to_constrained_optimized_vector = numpy.add(projected_vector, constrained_rejection_vector)
            # Make the constrained optimized state
            constrained_optimized_state = numpy.add(preceding_reference_waypoint, preceding_to_constrained_optimized_vector)
            # Return the constrained state
            return constrained_optimized_state

    def iterate(self, reference_path, current_path, optimization_bound, gradient_fn, verbose):
        #   Perform a single iteration of the optimization:
        #    1 - uniformly re-sample the path
        #    2 - compute the optimization hyperplane for each intermediate sample (start and end are left in place)
        #    3 - get the gradient for each sample
        #    4 - take a step along the gradient at each sample projected onto the hyperplane
        #    5 - ensure that all optimized points are within the optimization bound
        #    6 - ensure that no points have swapped places, and adjust if they have (not sure if needed, not implemented)
        # Resample first
        resampled_path = self.uniformly_resample_path(current_path)
        # Get the gradients for the intermediate points (padded with None on either end to match the full length)
        gradients = [None]
        for index in xrange(1, len(resampled_path) - 1):
            gradient = gradient_fn(resampled_path[index])
            gradients.append(gradient)
        gradients.append(None)
        # Go through and reject each gradient onto its respective step-defined hyperplane (padded with None)
        rejected_gradients = [None]
        for index in xrange(1, len(resampled_path) - 1):
            raw_gradient = gradients[index]
            previous_point = resampled_path[index - 1]
            current_point = resampled_path[index]
            next_point = resampled_path[index + 1]
            rejected_gradient = self.compute_gradient_rejection(previous_point, current_point, next_point, raw_gradient)
            rejected_gradients.append(rejected_gradient)
        rejected_gradients.append(None)
        # Compute a small gradient step for each intermediate point (padded with None)
        raw_optimization_steps = [None]
        for index in xrange(1, len(resampled_path) - 1):
            rejected_gradient = rejected_gradients[index]
            rejected_gradient_magnitude = numpy.linalg.norm(rejected_gradient)
            if abs(rejected_gradient_magnitude) <= abs(self.max_optimization_step_size):
                if verbose:
                    print("Rejected gradient is smaller than optimization step size, not limiting step")
                raw_optimization_steps.append(rejected_gradient)
            else:
                if verbose:
                    print("Limiting rejected gradient to optimization step size")
                limiting_ratio = abs(self.max_optimization_step_size / rejected_gradient_magnitude)
                limited_rejected_gradient = numpy.dot(rejected_gradient, limiting_ratio)
                raw_optimization_steps.append(limited_rejected_gradient)
        raw_optimization_steps.append(None)
        # Apply the gradient step to each of the intermediate points (first and last states are added unmodified)
        raw_optimized_path = [deepcopy(resampled_path[0])]
        for index in xrange(1, len(resampled_path) - 1):
            raw_sample = resampled_path[index]
            raw_optimization_step = raw_optimization_steps[index]
            raw_optimized_sample = numpy.add(raw_sample, raw_optimization_step)
            raw_optimized_path.append(raw_optimized_sample)
        raw_optimized_path.append(deepcopy(resampled_path[-1]))
        # Go back through the path and ensure each optimized point remains inside the optimization-bounded volume
        bounded_optimized_path = [deepcopy(raw_optimized_path[0])]
        for index in xrange(1, len(resampled_path) - 1):
            base_state = resampled_path[index]
            raw_optimized_state = raw_optimized_path[index]
            bounded_optimized_state = self.bound_optimization(raw_optimized_state, base_state, reference_path, optimization_bound)
            bounded_optimized_path.append(bounded_optimized_state)
        bounded_optimized_path.append(deepcopy(raw_optimized_path[-1]))
        # Compute the deltas for the current optimization step
        deltas = []
        initial_optimized_pairs = zip(resampled_path, bounded_optimized_path)
        for initial, optimized in initial_optimized_pairs:
            delta = self.weighted_euclidean_distance(initial, optimized)
            deltas.append(delta)
        # Return
        return [bounded_optimized_path, deltas]

    def check_convergence(self, deltas, verbose):
        #   Check for convergence from the magnitudes of the optimization deltas from the last iteration
        max_delta = max(deltas)
        total_delta = sum(deltas)
        if abs(total_delta) < abs(self.convergence_total_delta_threshold) and abs(max_delta) < abs(self.convergence_max_delta_threshold):
            if verbose:
                print("Convergence thresholds reached")
            return True
        if abs(total_delta) >= abs(self.convergence_total_delta_threshold):
            if verbose:
                print("Total delta outside of convergence bounds")
        if abs(max_delta) >= abs(self.convergence_max_delta_threshold):
            if verbose:
                print("Max delta outside of convergence bounds")
        return False

    def optimize(self, reference_path, init_path, optimization_bound, gradient_fn, verbose=True):
        #   We start the optimization process with:
        #    1 - a reference copy of the path
        #    2 - an initialization (in the simplest case, this is a deepcopy of the reference)
        #    3 - a bound on how far from the reference we can optimize
        #    4 - a function to compute the gradient at a given configuration
        #
        #   The actual optimization process is iterative - at every iteration, we:
        #    1 - uniformly re-sample the path
        #    2 - compute the optimization hyperplane for each sample
        #    3 - get the gradient for each sample
        #    4 - take a step along the gradient at each sample projected onto the hyperplane
        #    5 - ensure that all optimized points are within the optimization bound
        #    6 - check for convergence
        #
        #   The iterations continue until:
        #    1 - convergence is detected
        #    2 - we reach the maximum number of iterations set at optimizer initialization
        convergence_reached = False
        iterations = 0
        current_path = init_path
        # Run the optimizer until we've converged or run out of time
        while not convergence_reached and iterations < self.max_optimization_iterations:
            # Run a single iteration
            [current_path, deltas] = self.iterate(reference_path, current_path, optimization_bound, gradient_fn, verbose)
            # Check for convergence
            convergence_reached = self.check_convergence(deltas, verbose)
        # Check why we finished
        if not convergence_reached:
            if verbose:
                print("Maximum iterations limit reached, returning best effort")
        else:
            if verbose:
                print("Convergence reached, returning optimized path")
        # Return the optimized path
        return current_path
