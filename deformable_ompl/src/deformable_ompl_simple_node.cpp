#include "stdio.h"
#include <iostream>
#include <fstream>
#include <string>
#include "stdlib.h"
#include <chrono>
#include <ros/ros.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/util/RandomNumbers.h>
#include "deformable_ompl/simple_probe_state_space.hpp"
#include "deformable_ompl/simple_probe_dvxl_cost_fn.hpp"
#include "deformable_ompl/PlanSimplePath.h"
#include "deformable_ompl/ResampleSimplePath.h"
#include "deformable_ompl/SimplifySimplePath.h"
#include <visualization_msgs/MarkerArray.h>
#include "sdf_tools/collision_map.hpp"
#include "arc_utilities/pretty_print.hpp"

// Edge padding to match Bullet's edge effects
#define BULLET_EDGE_PADDING 0.125

// Macro to disable unused parameter compiler warnings
#define UNUSED(x) (void)(x)

using namespace deformable_ompl;

ros::Publisher g_debug_marker_pub;
DVXLGrid g_environment;

/*
 * Dummy deallocation functions when creating smart pointers from pointers to RAII'd objects rather than new'd objects
 */
void dealocate_StateValidityChecker_fn(ompl::base::StateValidityChecker* p)
{
    UNUSED(p);
}

void dealocate_MotionValidator_fn(ompl::base::MotionValidator* p)
{
    UNUSED(p);
}

void dealocate_OptimiztionObjective_fn(ompl::base::OptimizationObjective* p)
{
    UNUSED(p);
}

/*
 * Export a solution path to Rviz
 */
void DrawSolutionPath(ompl::base::PlannerStatus& status, std::vector<deformable_ompl::SimplePathState>& path, std::vector<std::pair<Eigen::Vector3d, DVXL>>& probe_points, std::string ns_suffix="")
{
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION || status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std_msgs::ColorRGBA color;
        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            color.r = 0.75;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;
        }
        else
        {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 1.0;
        }
        visualization_msgs::MarkerArray path_rep;
        // Loop through the path
        for (size_t idx = 0; idx < path.size(); idx++)
        {
            deformable_ompl::SimplePathState& probe_state = path[idx];
            Eigen::Vector3d position(probe_state.x, probe_state.y, 0.0);
            Eigen::Quaterniond orientation(Eigen::AngleAxisd(probe_state.yaw, Eigen::Vector3d::UnitZ()));
            // Make the display rep for the state
            visualization_msgs::Marker state_rep;
            // Populate the header
            state_rep.header.frame_id = "deformable_ompl";
            // Populate the options
            if (ns_suffix != "")
            {
                state_rep.ns = "path_" + ns_suffix;
            }
            else
            {
                state_rep.ns = "path";
            }
            state_rep.id = idx + 1;
            state_rep.type = visualization_msgs::Marker::CUBE_LIST;
            state_rep.action = visualization_msgs::Marker::ADD;
            state_rep.lifetime = ros::Duration(0.0);
            state_rep.frame_locked = false;
            state_rep.scale.x = DVXL_RESOLUTION;
            state_rep.scale.y = DVXL_RESOLUTION;
            state_rep.scale.z = DVXL_RESOLUTION;
            state_rep.pose.position.x = position.x();
            state_rep.pose.position.y = position.y();
            state_rep.pose.position.z = position.z();
            state_rep.pose.orientation.w = orientation.w();
            state_rep.pose.orientation.x = orientation.x();
            state_rep.pose.orientation.y = orientation.y();
            state_rep.pose.orientation.z = orientation.z();
            // Color the start and goal differently
            if (idx == 0)
            {
                std_msgs::ColorRGBA start_color;
                start_color.r = 1.0;
                start_color.g = 0.0;
                start_color.b = 0.0;
                start_color.a = 1.0;
                state_rep.color = start_color;
            }
            else if (idx == (path.size() - 1)  && status == ompl::base::PlannerStatus::EXACT_SOLUTION)
            {
                std_msgs::ColorRGBA end_color;
                end_color.r = 0.0;
                end_color.g = 1.0;
                end_color.b = 0.0;
                end_color.a = 1.0;
                state_rep.color = end_color;
            }
            else if (idx == (path.size() - 1)  && status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
            {
                std_msgs::ColorRGBA end_color;
                end_color.r = 1.0;
                end_color.g = 1.0;
                end_color.b = 0.0;
                end_color.a = 1.0;
                state_rep.color = end_color;
            }
            else
            {
                state_rep.color = color;
            }
            for (size_t pidx = 0; pidx < probe_points.size(); pidx++)
            {
                Eigen::Vector3d& point_location = probe_points[pidx].first;
                DVXL& point_dvxl = probe_points[pidx].second;
                if (point_dvxl.sensitivity > 0.0)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = point_location.x();
                    new_point.y = point_location.y();
                    new_point.z = point_location.z();
                    state_rep.points.push_back(new_point);
                }
            }
            // Add the marker
            path_rep.markers.push_back(state_rep);
        }
        // Publish
        g_debug_marker_pub.publish(path_rep);
    }
    else
    {
        ROS_ERROR("Only exact or approximate solutions can be drawn");
    }
}

/*
 * Discretize a cuboid obstacle to resolution - sized cells
 */
std::vector<std::pair<Eigen::Vector3d, DVXL>> DiscretizeObstacle(deformable_ompl::ObstacleConfig& obstacle, const double resolution, const double edge_padding)
{
    // First, pad the object size to match Bullet's edge effects
    obstacle.obstacle_size.x += edge_padding;
    obstacle.obstacle_size.y += edge_padding;
    obstacle.obstacle_size.z += edge_padding;
    std::vector<std::pair<Eigen::Vector3d, DVXL>> cells;
    int32_t x_cells = (int32_t)(obstacle.obstacle_size.x * 2.0 * (1.0 / resolution));
    int32_t y_cells = (int32_t)(obstacle.obstacle_size.y * 2.0 * (1.0 / resolution));
    int32_t z_cells = (int32_t)(obstacle.obstacle_size.z * 2.0 * (1.0 / resolution));
    DVXL obstacle_dvxl;
    obstacle_dvxl.deformability = obstacle.deformability;
    obstacle_dvxl.sensitivity = obstacle.sensitivity;
    obstacle_dvxl.mass = (obstacle.mass / (double)(x_cells * y_cells * z_cells));
    obstacle_dvxl.r = obstacle.color.r;
    obstacle_dvxl.g = obstacle.color.g;
    obstacle_dvxl.b = obstacle.color.b;
    obstacle_dvxl.a = obstacle.color.a;
    obstacle_dvxl.object_id = obstacle.obstacle_id;
    if (obstacle_dvxl.object_id == 0)
    {
        std::cerr << "Invalid object configuration" << std::endl;
    }
    for (int32_t xidx = 0; xidx < x_cells; xidx++)
    {
        for (int32_t yidx = 0; yidx < y_cells; yidx++)
        {
            for (int32_t zidx = 0; zidx < z_cells; zidx++)
            {
                double x_location = -(obstacle.obstacle_size.x - (resolution * 0.5)) + (resolution * xidx);
                double y_location = -(obstacle.obstacle_size.y - (resolution * 0.5)) + (resolution * yidx);
                double z_location = -(obstacle.obstacle_size.z - (resolution * 0.5)) + (resolution * zidx);
                Eigen::Vector3d cell_location(x_location, y_location, z_location);
                cells.push_back(std::pair<Eigen::Vector3d, DVXL>(cell_location, obstacle_dvxl));
            }
        }
    }
    return cells;
}

/*
 * Build a new environment from the provided obstacles
 */
DVXLGrid RebuildEnvironment(std::vector<deformable_ompl::ObstacleConfig>& obstacles, const double resolution, const bool add_padding)
{
    double edge_padding = 0.0;
    if (add_padding)
    {
        edge_padding = BULLET_EDGE_PADDING;
    }
    if (obstacles.empty())
    {
        ROS_WARN("No obstacles provided, not updating the environment");
        double grid_x_size = 1.0;
        double grid_y_size = 1.0;
        double grid_z_size = 1.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-0.5, -0.5, -0.5);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        DVXL default_dvxl;
        default_dvxl.deformability = 1.0;
        default_dvxl.sensitivity = 0.0;
        default_dvxl.mass = 0.0;
        default_dvxl.r = 0.0;
        default_dvxl.g = 0.0;
        default_dvxl.b = 0.0;
        default_dvxl.a = 0.0;
        default_dvxl.object_id = 0;
        DVXL oob_dvxl;
        oob_dvxl.deformability = 1.0;
        oob_dvxl.sensitivity = 0.0;
        oob_dvxl.mass = 0.0;
        oob_dvxl.r = 0.0;
        oob_dvxl.g = 0.0;
        oob_dvxl.b = 0.0;
        oob_dvxl.a = 0.0;
        oob_dvxl.object_id = 0;
        DVXLGrid grid(grid_origin_transform, "deformable_ompl", resolution, grid_x_size, grid_y_size, grid_z_size, default_dvxl, oob_dvxl);
        return grid;
    }
    else
    {
        ROS_INFO("Rebuilding the environment with %zu obstacles", obstacles.size());
        // We need to loop through the obstacles, discretize each obstacle, and then find the size of the grid we need to store them
        bool xyz_bounds_initialized = false;
        double x_min = 0.0;
        double y_min = 0.0;
        double z_min = 0.0;
        double x_max = 0.0;
        double y_max = 0.0;
        double z_max = 0.0;
        std::vector<std::pair<Eigen::Vector3d, DVXL>> all_obstacle_cells;
        for (size_t idx = 0; idx < obstacles.size(); idx++)
        {
            deformable_ompl::ObstacleConfig& obstacle = obstacles[idx];
            Eigen::Translation3d obstacle_translation(obstacle.obstacle_pose.position.x, obstacle.obstacle_pose.position.y, obstacle.obstacle_pose.position.z);
            Eigen::Quaterniond obstacle_rotation(obstacle.obstacle_pose.orientation.w, obstacle.obstacle_pose.orientation.x, obstacle.obstacle_pose.orientation.y, obstacle.obstacle_pose.orientation.z);
            Eigen::Affine3d obstacle_transform = obstacle_translation * obstacle_rotation;
            std::vector<std::pair<Eigen::Vector3d, DVXL>> obstacle_cells = DiscretizeObstacle(obstacle, resolution, edge_padding);
            for (size_t cidx = 0; cidx < obstacle_cells.size(); cidx++)
            {
                std::pair<Eigen::Vector3d, DVXL>& cell = obstacle_cells[cidx];
                Eigen::Vector3d& relative_location = cell.first;
                Eigen::Vector3d real_location = obstacle_transform * relative_location;
                std::pair<Eigen::Vector3d, DVXL> real_cell(real_location, cell.second);
                all_obstacle_cells.push_back(real_cell);
                // Check against the min/max extents
                if (xyz_bounds_initialized)
                {
                    if (real_location.x() < x_min)
                    {
                        x_min = real_location.x();
                    }
                    else if (real_location.x() > x_max)
                    {
                        x_max = real_location.x();
                    }
                    if (real_location.y() < y_min)
                    {
                        y_min = real_location.y();
                    }
                    else if (real_location.y() > y_max)
                    {
                        y_max = real_location.y();
                    }
                    if (real_location.z() < z_min)
                    {
                        z_min = real_location.z();
                    }
                    else if (real_location.z() > z_max)
                    {
                        z_max = real_location.z();
                    }
                }
                // If we haven't initialized the bounds yet, set them to the current position
                else
                {
                    x_min = real_location.x();
                    x_max = real_location.x();
                    y_min = real_location.y();
                    y_max = real_location.y();
                    z_min = real_location.z();
                    z_max = real_location.z();
                    xyz_bounds_initialized = true;
                }
            }
        }
        // Now that we've done that, we fill in the grid to store them
        // Key the center of the first cell off the the minimum value
        x_min -= (resolution * 0.5);
        y_min -= (resolution * 0.5);
        z_min -= (resolution * 0.5);
        // Add a 1-cell buffer to all sides
        x_min -= resolution;
        y_min -= resolution;
        z_min -= resolution;
        x_max += resolution;
        y_max += resolution;
        z_max += resolution;
        double grid_x_size = x_max - x_min;
        double grid_y_size = y_max - y_min;
        double grid_z_size = z_max - z_min;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(x_min, y_min, z_min);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        DVXL default_dvxl;
        default_dvxl.deformability = 1.0;
        default_dvxl.sensitivity = 0.0;
        default_dvxl.mass = 0.0;
        default_dvxl.r = 0.0;
        default_dvxl.g = 0.0;
        default_dvxl.b = 0.0;
        default_dvxl.a = 0.0;
        default_dvxl.object_id = 0;
        DVXL oob_dvxl;
        oob_dvxl.deformability = 1.0;
        oob_dvxl.sensitivity = 0.0;
        oob_dvxl.mass = 0.0;
        oob_dvxl.r = 0.0;
        oob_dvxl.g = 0.0;
        oob_dvxl.b = 0.0;
        oob_dvxl.a = 0.0;
        oob_dvxl.object_id = 0;
        DVXLGrid grid(grid_origin_transform, "deformable_ompl", resolution, grid_x_size, grid_y_size, grid_z_size, default_dvxl, oob_dvxl);
        // Fill it in
        for (size_t idx = 0; idx < all_obstacle_cells.size(); idx++)
        {
            Eigen::Vector3d& location = all_obstacle_cells[idx].first;
            DVXL& dvxl = all_obstacle_cells[idx].second;
            grid.Set(location.x(), location.y(), location.z(), dvxl);
        }
        // Set the environment
        return grid;
    }
}

/*
 * Service callback that plans a low-cost path using RRT* between the provided start config and goal region in the provided environment
 */
bool PlanSimplePathServiceCB(deformable_ompl::PlanSimplePath::Request& req, deformable_ompl::PlanSimplePath::Response& res)
{
    ROS_INFO("Processing PlanPath service...");
    g_environment = RebuildEnvironment(req.query.obstacles, COST_DVXL_RESOLUTION, false);
    visualization_msgs::Marker env_marker = g_environment.ExportForDisplay();
    visualization_msgs::MarkerArray env_rep;
    env_rep.markers.push_back(env_marker);
    DVXLGrid puncture_environment = RebuildEnvironment(req.query.obstacles, PUNCTURE_DVXL_RESOLUTION, false);
    visualization_msgs::Marker puncture_env_marker = puncture_environment.ExportForDisplay();
    puncture_env_marker.ns = "puncture_dvxl_grid";
    env_rep.markers.push_back(puncture_env_marker);
    g_debug_marker_pub.publish(env_rep);
    ROS_INFO("Setting up the planning problem...");
    // Set up the space for planning
    ompl::base::StateSpacePtr space(new SimpleProbeStateSpace());
    // Set the bounds of the space
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, ENV_X_SIZE);
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, ENV_Y_SIZE);
    bounds.setLow(2, ENV_MIN_YAW);
    bounds.setHigh(2, ENV_MAX_YAW);
    space->as<SimpleProbeStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));
    // Set up the collision checker
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> collision_planes;
    SimpleProbeDVXLCostValidityChecker dvxl_checker(space_information, g_environment, puncture_environment, collision_planes);
    //dvxl_checker.setVerbosity(true);
    std::vector<std::pair<Eigen::Vector3d, DVXL>> probe_points = dvxl_checker.GetProbePoints();
    ompl::base::StateValidityCheckerPtr dvxl_validity_checker(&dvxl_checker, dealocate_StateValidityChecker_fn);
    space_information->setStateValidityChecker(dvxl_validity_checker);
    // Set up the motion validity checker (this lets us check for punctures)
    ompl::base::MotionValidatorPtr dvxl_motion_validator(&dvxl_checker, dealocate_MotionValidator_fn);
    space_information->setMotionValidator(dvxl_motion_validator);
    // Finish setup of the space
    space_information->setup();
    // Create the problem definition
    ompl::base::ProblemDefinitionPtr problem_definition(new ompl::base::ProblemDefinition(space_information));
    // Make the start state
    ompl::base::ScopedState<> start(space);
    start->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->setPositionAndRotation(req.query.start.x, req.query.start.y, req.query.start.yaw);
    // Set the start state
    problem_definition->addStartState(start);
    // Set the goal type
    ompl::base::ScopedState<> goal(space);
    goal->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->setPositionAndRotation(req.query.goal.x, req.query.goal.y, req.query.goal.yaw);
    problem_definition->setGoalState(goal);
    // Setup the cost function
    if (req.query.p < 0.0 || req.query.p > 1.0)
    {
        res.result.status = deformable_ompl::PlanSimplePathResult::INVALID_PARAMETERS;
        return true;
    }
    // Make a cost function that combines path length with minimizing DVXL cost
    ompl::base::MultiOptimizationObjective* combined_cost_fn = new ompl::base::MultiOptimizationObjective(space_information);
    ompl::base::OptimizationObjectivePtr path_length_cost_fn(new ompl::base::PathLengthOptimizationObjective(space_information));
    ompl::base::OptimizationObjectivePtr dvxl_cost_fn(&dvxl_checker, dealocate_OptimiztionObjective_fn);
    combined_cost_fn->addObjective(path_length_cost_fn, (1.0 - req.query.p));
    combined_cost_fn->addObjective(dvxl_cost_fn, req.query.p);
    ompl::base::OptimizationObjectivePtr cost_fn(combined_cost_fn);
    problem_definition->setOptimizationObjective(cost_fn);
    // Make the planner
    ompl::base::PlannerPtr planner;
    if (req.query.planner_type == deformable_ompl::PlanSimplePathQuery::RRTSTAR)
    {
        planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(space_information));
    }
    else if (req.query.planner_type == deformable_ompl::PlanSimplePathQuery::TRRT)
    {
        planner = ompl::base::PlannerPtr(new ompl::geometric::TRRT(space_information));
    }
    else if (req.query.planner_type == deformable_ompl::PlanSimplePathQuery::PRMSTAR)
    {
        planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(space_information));
    }
    else
    {
        res.result.status = deformable_ompl::PlanSimplePathResult::INVALID_PARAMETERS;
        return true;
    }
    // Configure the planner
    planner->setProblemDefinition(problem_definition);
    planner->setup();
    // Attempt to solve, timing how long we take
    ROS_INFO("Planning...");
    struct timespec st, et;
    clock_gettime(CLOCK_MONOTONIC, &st);
    ompl::base::PlannerStatus status;
    try
    {
        status = planner->solve(req.query.time_limit);
    }
    catch (const std::exception& e)
    {
        std::cout << "Exception in planner: " << e.what() << std::endl;
        throw e;
    }
    clock_gettime(CLOCK_MONOTONIC, &et);
    // Get some statistics
    std::vector<u_int64_t> movement_counts = dvxl_checker.GetValidInvalidMovementCounts();
    ROS_INFO("DVXL Checker evaluated %lu valid movements and %lu invalid movements, of invalid movements, %lu were state-invalid and %lu were puncture-invalid, %lu fast-invalid", movement_counts[0], movement_counts[1], movement_counts[2], movement_counts[3], movement_counts[4]);
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION || status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        // Get the solution path
        boost::shared_ptr<ompl::geometric::PathGeometric> path = boost::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition->getSolutionPath());
        // Pack the solution path into the response
        res.result.path.clear();
        std::vector<ompl::base::State*> states = path->getStates();
        // Add the very first state
        deformable_ompl::SimpleProbeStateSpace::StateType* first_probe_state = states[0]->as<deformable_ompl::SimpleProbeStateSpace::StateType>();
        Eigen::Vector3d first_state_position = first_probe_state->getEigenPosition();
        deformable_ompl::SimplePathState first_path_state;
        first_path_state.x = first_state_position.x();
        first_path_state.y = first_state_position.y();
        first_path_state.yaw = first_probe_state->getRotation();
        first_path_state.deformation_cost = dvxl_cost_fn->stateCost(first_probe_state).GET_OMPL_COST;
        first_path_state.length_cost = path_length_cost_fn->stateCost(first_probe_state).GET_OMPL_COST;
        res.result.path.push_back(first_path_state);
        // Add the following states
        for (size_t idx = 1; idx < states.size(); idx++)
        {
            // Get the start and end states of this section of the path
            deformable_ompl::SimpleProbeStateSpace::StateType* start_probe_state = states[idx - 1]->as<deformable_ompl::SimpleProbeStateSpace::StateType>();
            deformable_ompl::SimpleProbeStateSpace::StateType* end_probe_state = states[idx]->as<deformable_ompl::SimpleProbeStateSpace::StateType>();
            // We want to add all the intermediate states to our returned path
            unsigned int num_segments = space_information->getStateSpace()->validSegmentCount(start_probe_state, end_probe_state);
            // If there's only one segment, we just add the end state of the window
            if (num_segments <= 1)
            {
                Eigen::Vector3d position = end_probe_state->getEigenPosition();
                deformable_ompl::SimplePathState path_state;
                path_state.x = position.x();
                path_state.y = position.y();
                path_state.yaw = end_probe_state->getRotation();
                path_state.deformation_cost = dvxl_cost_fn->stateCost(end_probe_state).GET_OMPL_COST;
                path_state.length_cost = path_length_cost_fn->stateCost(end_probe_state).GET_OMPL_COST;
                res.result.path.push_back(path_state);
            }
            // If there is more than one segment, interpolate between the start and end
            else
            {
                ompl::base::State* intermediate_state = space_information->allocState();
                for (unsigned int idx = 1; idx <= num_segments; idx++)
                {
                    // Interpolate the intermediate state
                    space_information->getStateSpace()->interpolate(start_probe_state, end_probe_state, (double) idx / (double) num_segments, intermediate_state);
                    // Extract and pack the intermediate state
                    Eigen::Vector3d position = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getEigenPosition();
                    deformable_ompl::SimplePathState path_state;
                    path_state.x = position.x();
                    path_state.y = position.y();
                    path_state.yaw = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getRotation();
                    path_state.deformation_cost = dvxl_cost_fn->stateCost(intermediate_state).GET_OMPL_COST;
                    path_state.length_cost = path_length_cost_fn->stateCost(intermediate_state).GET_OMPL_COST;
                    res.result.path.push_back(path_state);
                }
                space_information->freeState(intermediate_state);
            }
        }
        // Check the topology-correctness of the final planned path
        //bool topology_valid = dvxl_checker.CheckFinalPath(res.path);
        bool topology_valid = dvxl_checker.IncrementalCheckFinalPath(res.result.path, g_debug_marker_pub);
        if (!topology_valid)
        {
            status = ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
        }
        // Draw the path in Rviz
        DrawSolutionPath(status, res.result.path, probe_points);
        // Set the runtime, cost, and length
        double secs = (double)(et.tv_sec - st.tv_sec);
        secs = secs + ((double)(et.tv_nsec - st.tv_nsec) / 1000000000.0);
        res.result.run_time = secs;
        res.result.total_cost = path->cost(cost_fn).GET_OMPL_COST;
        res.result.total_length = path->length();
        // Set the status and return
        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            res.result.status = deformable_ompl::PlanSimplePathResult::SUCCESS;
            ROS_INFO("...returning successful plan");
        }
        else
        {
            res.result.status = deformable_ompl::PlanSimplePathResult::APPROXIMATE;
            ROS_ERROR("...returning unsuccessful approximate plan");
        }
        return true;
    }
    else if (status == ompl::base::PlannerStatus::INVALID_START)
    {
        // Return invalid start error status
        res.result.status = deformable_ompl::PlanSimplePathResult::INVALID_START;
        ROS_ERROR("...invalid start");
        return true;
    }
    else if (status == ompl::base::PlannerStatus::INVALID_GOAL)
    {
        // Return invalid goal error status
        res.result.status = deformable_ompl::PlanSimplePathResult::INVALID_GOAL;
        ROS_ERROR("...invalid goal");
        return true;
    }
    else if (status == ompl::base::PlannerStatus::TIMEOUT)
    {
        // Return timeout error status
        res.result.status = deformable_ompl::PlanSimplePathResult::TIMEOUT;
        ROS_ERROR("...timeout");
        return true;
    }
    else
    {
        // Return unknown error status
        res.result.status = deformable_ompl::PlanSimplePathResult::UNKNOWN_ERROR;
        ROS_ERROR("...unknown error");
        return true;
    }
}

inline int64_t snap(const double val)
{
    if (val > 0.0)
    {
        return (int64_t)(val + 0.5);
    }
    else if (val < -0.0)
    {
        return (int64_t)(val - 0.5);
    }
    else
    {
        return 0;
    }
}

bool ResampleSimplePathServiceCB(deformable_ompl::ResampleSimplePath::Request& req, deformable_ompl::ResampleSimplePath::Response& res)
{
    ROS_INFO("Processing ResamplePath service...");
    if (req.query.original_path.size() == 0)
    {
        ROS_WARN("Original path empty, no resampling");
        return true;
    }
    else if (req.query.original_path.size() == 1)
    {
        ROS_WARN("Original path has 1 state, no resampling");
        res.result.resampled_path.clear();
        res.result.resampled_path.push_back(req.query.original_path[0]);
    }
    ROS_INFO("Setting up the space...");
    // Set up the space for planning
    ompl::base::StateSpacePtr space(new SimpleProbeStateSpace());
    // Set the bounds of the space
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, ENV_X_SIZE);
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, ENV_Y_SIZE);
    bounds.setLow(2, ENV_MIN_YAW);
    bounds.setHigh(2, ENV_MAX_YAW);
    space->as<SimpleProbeStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));
    ROS_INFO("Resampling path...");
    // Clear the resampled path
    res.result.resampled_path.clear();
    // Copy the first state over
    res.result.resampled_path.push_back(req.query.original_path[0]);
    // Loop through the path, adding interpolated states as needed
    for (size_t idx = 1; idx < req.query.original_path.size(); idx++)
    {
        // Get the raw path states
        const deformable_ompl::SimplePathState& prev_state = req.query.original_path[idx - 1];
        const deformable_ompl::SimplePathState& curr_state = req.query.original_path[idx];
        // Convert them to OMPL states
        ompl::base::State* prev_probe_state = space_information->allocState();
        prev_probe_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->setPositionAndRotation(prev_state.x, prev_state.y, prev_state.yaw);
        ompl::base::State* curr_probe_state = space_information->allocState();
        curr_probe_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->setPositionAndRotation(curr_state.x, curr_state.y, curr_state.yaw);
        // We want to add all the intermediate states to our returned path
        double distance = space_information->getStateSpace()->distance(prev_probe_state, curr_probe_state);
        double raw_num_intervals = distance / req.query.interval_size;
        int64_t num_segments = snap(raw_num_intervals);
        // If there's only one segment, we just add the end state of the window
        if (num_segments <= 1)
        {
            res.result.resampled_path.push_back(curr_state);
        }
        // If there is more than one segment, interpolate between the start and end (including the end)
        else
        {
            ompl::base::State* intermediate_state = space_information->allocState();
            for (int64_t idx = 1; idx <= num_segments; idx++)
            {
                // Interpolate the intermediate state
                space_information->getStateSpace()->interpolate(prev_probe_state, curr_probe_state, (double) idx / (double) num_segments, intermediate_state);
                // Extract and pack the intermediate state
                deformable_ompl::SimplePathState path_state;
                path_state.x = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getX();
                path_state.y = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getY();
                path_state.yaw = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getYaw();
                res.result.resampled_path.push_back(path_state);
            }
            space_information->freeState(intermediate_state);
        }
        space_information->freeState(prev_probe_state);
        space_information->freeState(curr_probe_state);
    }
    ROS_INFO("Resampled original path with %zu states to %zu states", req.query.original_path.size(), res.result.resampled_path.size());
    return true;
}

std::vector<deformable_ompl::SimplePathState> ShortcutPath(const std::vector<deformable_ompl::SimplePathState>& original_path, const double max_shortcut_fraction, const ompl::base::SpaceInformationPtr& space_information, ompl::RNG& prng)
{
    std::vector<deformable_ompl::SimplePathState> shortcut_path;
    // Pick a random start index
    int32_t base_index = prng.uniformInt(0, (original_path.size() - 1)); // We know this is safe
    double offset_fraction = prng.uniformReal(-max_shortcut_fraction, max_shortcut_fraction);
    int32_t raw_offset_index = base_index + (int32_t)((double)original_path.size() * offset_fraction); // Could be out of bounds
    int32_t safe_offset_index = std::max(0, std::min(raw_offset_index, (int32_t)(original_path.size() - 1))); // Make sure it's in bounds
    // Get the start and end indices
    u_int32_t start_index = (u_int32_t)std::min(base_index, safe_offset_index);
    u_int32_t end_index = (u_int32_t)std::max(base_index, safe_offset_index);
    // Copy the first part of the original path
    shortcut_path.insert(shortcut_path.end(), original_path.begin(), original_path.begin() + start_index);
    // Interpolate between start and end index
    // Get the raw path states
    const deformable_ompl::SimplePathState& start_state = original_path[start_index];
    const deformable_ompl::SimplePathState& end_state = original_path[end_index];
    // Convert them to OMPL states
    ompl::base::State* start_probe_state = space_information->allocState();
    start_probe_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->setPositionAndRotation(start_state.x, start_state.y, start_state.yaw);
    ompl::base::State* end_probe_state = space_information->allocState();
    end_probe_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->setPositionAndRotation(end_state.x, end_state.y, end_state.yaw);
    // Compute the number of intermediate states
    unsigned int num_segments = space_information->getStateSpace()->validSegmentCount(start_probe_state, end_probe_state);
    if (num_segments <= 1)
    {
        shortcut_path.push_back(start_state);
        // Copy the remainder of the path
        shortcut_path.insert(shortcut_path.end(), original_path.begin() + end_index, original_path.end());
    }
    else
    {
        // Add the intermediate states between start and end
        ompl::base::State* intermediate_state = space_information->allocState();
        for (unsigned int idx = 0; idx < num_segments; idx++)
        {
            // Interpolate the intermediate state
            space_information->getStateSpace()->interpolate(start_probe_state, end_probe_state, (double) idx / (double) num_segments, intermediate_state);
            // Extract and pack the intermediate state
            deformable_ompl::SimplePathState path_state;
            path_state.x = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getX();
            path_state.y = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getY();
            path_state.yaw = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getYaw();
            shortcut_path.push_back(path_state);
        }
        space_information->freeState(intermediate_state);
        // Copy the remainder of the path
        shortcut_path.insert(shortcut_path.end(), original_path.begin() + end_index, original_path.end());
    }
    space_information->freeState(start_probe_state);
    space_information->freeState(end_probe_state);
    return shortcut_path;
}

bool SimplifySimplePathServiceCB(deformable_ompl::SimplifySimplePath::Request& req, deformable_ompl::SimplifySimplePath::Response& res)
{
    ROS_INFO("Processing SimplifyPath service...");
    if (req.query.max_shortcut_fraction < 0.0 || req.query.max_shortcut_fraction > 1.0)
    {
        ROS_ERROR("Invalid max_shortcut_fraction - it must be between 0 and 1");
        res.result.status = deformable_ompl::SimplifySimplePathResult::INVALID_PARAMETERS;
        return true;
    }
    if (req.query.original_path.size() < 3)
    {
        ROS_INFO("Path is already as simple as possible - no simplification will be performed");
        res.result.simplified_path = req.query.original_path;
        res.result.status = deformable_ompl::SimplifySimplePathResult::SUCCESS;
        return true;
    }
    g_environment = RebuildEnvironment(req.query.obstacles, COST_DVXL_RESOLUTION, false);
    visualization_msgs::Marker env_marker = g_environment.ExportForDisplay();
    visualization_msgs::MarkerArray env_rep;
    env_rep.markers.push_back(env_marker);
    DVXLGrid puncture_environment = RebuildEnvironment(req.query.obstacles, PUNCTURE_DVXL_RESOLUTION, false);
    visualization_msgs::Marker puncture_env_marker = puncture_environment.ExportForDisplay();
    puncture_env_marker.ns = "puncture_dvxl_grid";
    env_rep.markers.push_back(puncture_env_marker);
    g_debug_marker_pub.publish(env_rep);
    ROS_INFO("Setting up the simplifier...");
    // Set up the space for planning
    ompl::base::StateSpacePtr space(new SimpleProbeStateSpace());
    // Set the bounds of the space
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, ENV_X_SIZE);
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, ENV_Y_SIZE);
    bounds.setLow(2, ENV_MIN_YAW);
    bounds.setHigh(2, ENV_MAX_YAW);
    space->as<SimpleProbeStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));
    // Set up the collision checker
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> collision_planes;
    SimpleProbeDVXLCostValidityChecker dvxl_checker(space_information, g_environment, puncture_environment, collision_planes);
    // Make a RNG
    ompl::RNG prng;
    ROS_INFO("Setup complete, simplifing for up to %u iterations (or until %u failed simplifications)", req.query.max_iterations, req.query.max_failed_iterations);
    struct timespec st, et;
    clock_gettime(CLOCK_MONOTONIC, &st);
    // Resample the original path to make it denser
    deformable_ompl::ResampleSimplePath::Request resample_req;
    resample_req.query.original_path = req.query.original_path;
    resample_req.query.interval_size = 0.005;
    deformable_ompl::ResampleSimplePath::Response resample_res;
    ResampleSimplePathServiceCB(resample_req, resample_res);
    // Compute stats
    double original_length = dvxl_checker.ComputePathLength(resample_res.result.resampled_path);
    double original_dvxl_cost = dvxl_checker.ComputeCostFinalPath(resample_res.result.resampled_path, g_debug_marker_pub);
    // Store state
    double current_length = original_length;
    double current_dvxl_cost = original_dvxl_cost;
    std::vector<deformable_ompl::SimplePathState> current_path = resample_res.result.resampled_path;
    u_int32_t num_iterations = 0;
    u_int32_t failed_iterations = 0;
    while (num_iterations < req.query.max_iterations && failed_iterations < req.query.max_failed_iterations && current_path.size() > 2)
    {
        std::cout << "Shortcut iteration " << num_iterations << std::endl;
        // Shortcut the current path
        std::vector<deformable_ompl::SimplePathState> raw_shortcut_path = ShortcutPath(current_path, req.query.max_shortcut_fraction, space_information, prng);
        deformable_ompl::ResampleSimplePath::Request resample_shortcut_req;
        resample_shortcut_req.query.original_path = raw_shortcut_path;
        resample_shortcut_req.query.interval_size = 0.005;
        deformable_ompl::ResampleSimplePath::Response resample_shortcut_res;
        ResampleSimplePathServiceCB(resample_shortcut_req, resample_shortcut_res);
        std::vector<deformable_ompl::SimplePathState> shortcut_path = resample_shortcut_res.result.resampled_path;
        // Check if we actually made it shorter
        double shortcut_length = dvxl_checker.ComputePathLength(shortcut_path);
        if (shortcut_length >= current_length)
        {
            num_iterations++;
            failed_iterations++;
        }
        else
        {
            // Check the cost of the path
            double shortcut_cost = dvxl_checker.ComputeCostFinalPath(shortcut_path, g_debug_marker_pub);
            // Check the validity of the path
            bool valid = dvxl_checker.IncrementalCheckFinalPath(shortcut_path, g_debug_marker_pub);
            if (valid && shortcut_cost <= current_dvxl_cost)
            {
                current_length = shortcut_length;
                current_dvxl_cost = shortcut_cost;
                current_path = shortcut_path;
                num_iterations++;
                failed_iterations = 0;
            }
            else
            {
                num_iterations++;
                failed_iterations++;
            }
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &et);
    double secs = (double)(et.tv_sec - st.tv_sec);
    secs = secs + ((double)(et.tv_nsec - st.tv_nsec) / 1000000000.0);
    ROS_INFO("Simplification of %u iterations took %f seconds", num_iterations, secs);
    res.result.simplified_path = current_path;
    // Compute final stats
    double simplified_length = dvxl_checker.ComputePathLength(res.result.simplified_path);
    double simplfied_dvxl_cost = dvxl_checker.ComputeCostFinalPath(res.result.simplified_path, g_debug_marker_pub);
    ROS_INFO("Simplified original path with %zu states, length %f, DVXL cost %f to %zu states, length %f, DVXL cost %f", req.query.original_path.size(), original_length, original_dvxl_cost, res.result.simplified_path.size(), simplified_length, simplfied_dvxl_cost);
    res.result.status = deformable_ompl::SimplifySimplePathResult::SUCCESS;
    ROS_INFO("Drawing path in RVIZ...");
    ompl::base::PlannerStatus dummy_status = ompl::base::PlannerStatus::EXACT_SOLUTION;
    std::vector<std::pair<Eigen::Vector3d, DVXL>> probe_points = dvxl_checker.GetProbePoints();
    DrawSolutionPath(dummy_status, res.result.simplified_path, probe_points, "simplified");
    return true;
}

/*
 * Make the default environment
 */
DVXLGrid MakeDefaultEnvironment()
{
    DVXL default_dvxl;
    default_dvxl.deformability = 1.0;
    default_dvxl.sensitivity = 0.0;
    default_dvxl.mass = 0.0;
    default_dvxl.r = 0x00;
    default_dvxl.g = 0x00;
    default_dvxl.b = 0x00;
    default_dvxl.a = 0x00;
    DVXL oob_dvxl;
    oob_dvxl.deformability = 1.0;
    oob_dvxl.sensitivity = 0.0;
    oob_dvxl.mass = 0.0;
    oob_dvxl.r = 0x00;
    oob_dvxl.g = 0x00;
    oob_dvxl.b = 0x00;
    oob_dvxl.a = 0x00;
    DVXLGrid environment("deformable_ompl", DVXL_RESOLUTION, 0.25, 0.25, 0.25, default_dvxl, oob_dvxl);
    for (int64_t x_index = 0; x_index < environment.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < environment.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < environment.GetNumZCells(); z_index++)
            {
                std::vector<double> location = environment.GridIndexToLocation(x_index, y_index, z_index);
                if (location[0] < 2.5 && location[0] > -2.5)
                {
                    if (location[2] < 10.0)
                    {
                        DVXL obstacle_dvxl;
                        obstacle_dvxl.deformability = 0.1;
                        obstacle_dvxl.sensitivity = 1.0;
                        obstacle_dvxl.mass = 0.0;
                        obstacle_dvxl.r = 0x00;
                        obstacle_dvxl.g = 0x00;
                        obstacle_dvxl.b = 0xff;
                        obstacle_dvxl.a = 0xff;
                        environment.Set(x_index, y_index, z_index, obstacle_dvxl);
                    }
                    else
                    {
                        DVXL obstacle_dvxl;
                        obstacle_dvxl.deformability = 0.1;
                        obstacle_dvxl.sensitivity = 0.25;
                        obstacle_dvxl.mass = 0.0;
                        obstacle_dvxl.r = 0xff;
                        obstacle_dvxl.g = 0xff;
                        obstacle_dvxl.b = 0x00;
                        obstacle_dvxl.a = 0xff;
                        environment.Set(x_index, y_index, z_index, obstacle_dvxl);
                    }
                }
            }
        }
    }
    return environment;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deformable_ompl_simple_node");
    ROS_INFO("Starting deformable OMPL simple planner node...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string environment_path;
    nhp.param(std::string("environment_path"), environment_path, std::string(""));
    if (environment_path == "")
    {
        ROS_INFO("No environment path provided, making default environment...");
        g_environment = MakeDefaultEnvironment();
        ROS_INFO("...made");
    }
    else
    {
        ROS_INFO("Attempting to load environment from path - %s ...", environment_path.c_str());
        bool loaded = g_environment.LoadFromFile(environment_path);
        if (loaded)
        {
            ROS_INFO("...loaded");
        }
        else
        {
            ROS_FATAL("...loading failed");
            ros::shutdown();
            exit(1);
        }
    }
    ROS_INFO("Sending environment to RVIZ...");
    g_debug_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName() + "/debug_markers", 1, true);
    ROS_INFO("Starting planning server...");
    ros::ServiceServer planner_server = nh.advertiseService(ros::this_node::getName() + "/plan_simple_path", PlanSimplePathServiceCB);
    ros::ServiceServer resample_server = nh.advertiseService(ros::this_node::getName() + "/resample_simple_path", ResampleSimplePathServiceCB);
    ros::ServiceServer simplify_server = nh.advertiseService(ros::this_node::getName() + "/simplify_simple_path", SimplifySimplePathServiceCB);
    ros::Rate spin_rate(1.0);
    while (ros::ok())
    {
        visualization_msgs::Marker env_marker = g_environment.ExportForDisplay();
        visualization_msgs::MarkerArray env_rep;
        env_rep.markers.push_back(env_marker);
        g_debug_marker_pub.publish(env_rep);
        ros::spinOnce();
        spin_rate.sleep();
    }
    ROS_INFO("...planning server exiting");
    return 0;
}
