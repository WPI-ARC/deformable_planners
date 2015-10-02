#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <random>
#include "arc_utilities/eigen_helpers.hpp"
#include "arc_utilities/pretty_print.hpp"
#include "deformable_ompl/simple_pid_controller.hpp"
#include "deformable_ompl/simple_uncertainty_models.hpp"
#include "deformable_ompl/simple_rrt_planner.hpp"

#ifndef NOMDP_CONTACT_PLANNING_HPP
#define NOMDP_CONTACT_PLANNING_HPP

;
// Make a simulation system that steps the robot at one step at a time
// Decide if a particle is stuck "dead" or "alive"
// Make a set of helper functions that package the simulator calls into the motion planner interface
// Make a good approximate distance fn -> EMD isn't used in practical cases where the distributions don't overlap (which is probably true for us)
// We need a distance function that captures:
// - proximity (distance between target and expectation)
// - certainty (1/spread of particles)
// - success (Pfeasible)

#endif // NOMDP_CONTACT_PLANNING_HPP
