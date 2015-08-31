#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   Python/NUMPY implementation of GRC:             #
#   "Gradient Rejection Control"                    #
#                                                   #
#####################################################

import math
import numpy


class GradientRejectionController(object):

    def __init__(self):
        pass

    def compute_rejection(self, current_ideal_state, current_target_state, current_gradient, correction_bound, verbose):
        # Compute the "target" vector to the target state
        next_step_vector = numpy.subtract(current_target_state, current_ideal_state)
        if verbose:
            print("Vector current_ideal->current_target is " + str(next_step_vector) + " [GRC]")
        # Compute the projection of the current gradient onto the next_step_vector
        projected_gradient = numpy.dot(next_step_vector, numpy.dot(current_gradient, next_step_vector) / numpy.dot(next_step_vector, next_step_vector))
        if verbose:
            print("Projected gradient vector is " + str(projected_gradient) + " [GRC]")
        # Compute the rejection of the current gradient onto the next_step_vector
        rejected_gradient = numpy.subtract(current_gradient, projected_gradient)
        if verbose:
            print("Rejected gradient vector is " + str(rejected_gradient) + " (unbounded) [GRC]")
        # Limit the magnitude of the rejected gradient
        unbounded_correction_magnitude = numpy.linalg.norm(rejected_gradient)
        if abs(unbounded_correction_magnitude) > abs(correction_bound):
            correction_ratio = abs(correction_bound / unbounded_correction_magnitude)
            if verbose:
                print("Scaling down rejected gradient to match correction bounds with ratio " + str(correction_ratio) + " [GRC]")
            rejected_gradient = numpy.dot(rejected_gradient, correction_ratio)
        if verbose:
            print("Rejected gradient is " + str(rejected_gradient) + " (bounded) [GRC]")
        # Combine the target vector and correction
        corrected_target_state = numpy.add(current_target_state, rejected_gradient)
        if verbose:
            print("Combined gradient with target state to produce corrected target " + str(corrected_target_state) + " [GRC]")
        return corrected_target_state

    def angle_between_vectors(self, vector1, vector2):
        unit_vector1 = vector1 / numpy.linalg.norm(vector1)
        unit_vector2 = vector2 / numpy.linalg.norm(vector2)
        angle = math.acos(numpy.dot(unit_vector1, unit_vector2))
        if math.isnan(angle):
            if (unit_vector1 == unit_vector2).all():
                angle = 0.0
            else:
                angle = numpy.pi
        return angle

    def compute_back_projection(self, current_ideal_state, current_target_state, next_target_state, naive_new_state, verbose):
        # First, we check to see if we need back projection
        # if the dot product of 1 and 2 is negative, then we need to back-project
        # 1-vector from next target state to naive new state
        # 2-vector from next target state to current target state
        next_naive_vector = numpy.subtract(naive_new_state, next_target_state)
        next_current_vector = numpy.subtract(current_target_state, next_target_state)
        dot_product = numpy.dot(next_naive_vector, next_current_vector)
        if dot_product >= -0.0:
            if verbose:
                print("No back projection required [GRC]")
            return naive_new_state
        if verbose:
            print("Computing back-projected constraints with dot product " + str(dot_product) + " [GRC]")
        # Apply back projection
        # Compute 'alpha2', the angle between current_target->current_ideal and current_target->next_target
        current_ideal_vector = numpy.subtract(current_ideal_state, current_target_state)
        current_next_vector = numpy.subtract(next_target_state, current_target_state)
        alpha2 = self.angle_between_vectors(current_ideal_vector, current_next_vector)
        if abs(alpha2) < 0.001:
            if verbose:
                print("Angle " + str(alpha2) + " between target->ideal and target->next is almost zero - this is an error condition [GRC]")
            return naive_new_state
        if (abs(alpha2) - numpy.pi) < 0.001:
            if verbose:
                print("Angle " + str(alpha2) + " between target->ideal and target->next is almost pi - this is an error condition [GRC]")
            return naive_new_state
        if (abs(alpha2) - (numpy.pi / 2.0)) < 0.001:
            if verbose:
                print("Angle " + str(alpha2) + " between target->ideal and target->next is almost pi/2 - the farthest admissible correction is the next state [GRC]")
            return next_target_state
        if abs(alpha2) > (numpy.pi / 2.0):
            angle = alpha2 - (numpy.pi / 2.0)
            if verbose:
                print("Angle " + str(alpha2) + " between target->ideal and target->next is greater than pi/2 - computing correction directly with alpha = " + str(angle) + " [GRC]")
            # Compute the maximum correction using law of sines
            # Side "c"
            current_next_magnitude = numpy.linalg.norm(current_next_vector)
            # Side "b" = "c" / sin(angle "C")
            max_correction_magnitude = current_next_magnitude / math.sin((numpy.pi / 2.0) - angle)
            # Get the correction vector
            correction_vector = numpy.subtract(naive_new_state, current_target_state)
            # Get the unit vector of it
            correction_unit_vector = correction_vector / numpy.linalg.norm(correction_vector)
            # Scale the unit vector by the max_correction_vector
            max_correction_vector = correction_unit_vector * max_correction_magnitude
            # Add the max_correction_vector to the current target to produce the corrected target
            corrected_target_state = numpy.add(current_target_state, max_correction_vector)
            return corrected_target_state
        if abs(alpha2) < (numpy.pi / 2.0):
            angle = numpy.pi - alpha2
            if verbose:
                print("Angle " + str(alpha2) + " between target->ideal and target->next is less that pi/2 - computing with pi/2 - alpha = " + str(angle) + " [GRC]")
            # Compute the maximum correction using law of sines
            # Side "c"
            current_next_magnitude = numpy.linalg.norm(current_next_vector)
            # Side "b" = "c" / sin(angle "C")
            max_correction_magnitude = current_next_magnitude / math.sin((numpy.pi / 2.0) - angle)
            # Get the correction vector
            correction_vector = numpy.subtract(naive_new_state, current_target_state)
            # Get the unit vector of it
            correction_unit_vector = correction_vector / numpy.linalg.norm(correction_vector)
            # Scale the unit vector by the max_correction_vector
            max_correction_vector = correction_unit_vector * max_correction_magnitude
            # Add the max_correction_vector to the current target to produce the corrected target
            corrected_target_state = numpy.add(current_target_state, max_correction_vector)
            return corrected_target_state
        if verbose:
            print("ERROR - You should not be here, something horrible has gone wrong! [GRC]")
        raise Exception("GRC numerical error")

    def compute_new_target(self, current_ideal, current_state, current_target, next_target, current_gradient, correction_bound, verbose=True):
        if verbose:
            print("----------")
            print("Should be at " + str(current_ideal) + " [GRC]")
            print("Currently at " + str(current_state) + " [GRC]")
            print("Current gradient " + str(current_gradient) + " [GRC]")
            print("Current target " + str(current_target) + " [GRC]")
            print("Next target will be " + str(next_target) + " [GRC]")
            print("GRC correction bound " + str(correction_bound) + " [GRC]")
        # First, compute the naive correction
        naive_new_state = self.compute_rejection(current_ideal, current_target, current_gradient, correction_bound, verbose)
        if verbose:
            print("New target without constraint propagation " + str(naive_new_state) + " [GRC]")
        # Now, back-project the constraints from the *next* state back onto the current correction
        corrected_target = self.compute_back_projection(current_ideal, current_target, next_target, naive_new_state, verbose)
        if verbose:
            print("New target with constraint propagation " + str(corrected_target) + " [GRC]")
        return corrected_target
