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
#include "deformable_ompl/probe_state_space.hpp"
#include "deformable_ompl/probe_dvxl_cost_fn.hpp"
#include "deformable_ompl/PlanPath.h"
#include "deformable_ompl/ComputeDVXLCost.h"
#include "deformable_ompl/ResamplePath.h"
#include "deformable_ompl/SimplifyPath.h"
#include "deformable_ompl/ComputeTrialSetup.h"
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
 * Simply test if a sampled probe state falls within the goal region (not very successful - very few goal states get generated this way)
 */
class ProbeGoalRegion : public ompl::base::GoalRegion
{
protected:

    Eigen::Vector3d target_point_;
    double tolerance_;

public:

    ProbeGoalRegion(const ompl::base::SpaceInformationPtr& space_information, Eigen::Vector3d target_point, double tolerance) : ompl::base::GoalRegion(space_information)
    {
        target_point_ = target_point;
        tolerance_ = fabs(tolerance);
        setThreshold(1e-2);
    }

    virtual double distanceGoal(const ompl::base::State* state) const
    {
        // Convert the state to our state type
        const deformable_ompl::ProbeStateSpace::StateType* probe_state = state->as<deformable_ompl::ProbeStateSpace::StateType>();
        // Get the transform of the probe
        Eigen::Affine3d probe_transform = probe_state->getEigenTransform();
        // Make the tip offset
        Eigen::Vector3d probe_tip_offset(0.0, 0.0, PROBE_LENGTH * 0.5);
        // Get the probe tip position
        Eigen::Vector3d probe_tip_position = probe_transform * probe_tip_offset;
        // Check the R^3 Euclidean distance to the target point
        Eigen::Vector3d error_vector = target_point_ - probe_tip_position;
        double error = error_vector.norm();
        // Return the distance to the goal state
        return error - tolerance_;
    }
};

/*
 * Provide a way to sample goal states from the provided goal region+tolerance (very successful - lots of states are generated)
 */
class SampleableProbeGoalRegion : public ompl::base::GoalSampleableRegion
{
protected:

    Eigen::Vector3d target_point_;
    double tolerance_;
    ompl::base::StateSpacePtr r3_space_;
    ompl::base::SpaceInformationPtr r3_space_information_;
    mutable ompl::base::ValidStateSamplerPtr r3_sampler_;
    ompl::base::StateSpacePtr so3_space_;
    ompl::base::SpaceInformationPtr so3_space_information_;
    mutable ompl::base::ValidStateSamplerPtr so3_sampler_;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> collision_planes_;

public:

    SampleableProbeGoalRegion(const ompl::base::SpaceInformationPtr& space_information, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& collision_planes, Eigen::Vector3d target_point, double tolerance) : ompl::base::GoalSampleableRegion(space_information)
    {
        collision_planes_ = collision_planes;
        target_point_ = target_point;
        tolerance_ = fabs(tolerance);
        setThreshold(1e-2);
        // Make samplers for R3 and SO(3)
        // First, we have to make the individual spaces
        r3_space_ = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(3));
        ompl::base::RealVectorBounds r3_space_bounds(3);
        r3_space_bounds.setLow(0, target_point.x() - tolerance);
        r3_space_bounds.setHigh(0, target_point.x() + tolerance);
        r3_space_bounds.setLow(1, target_point.y() - tolerance);
        r3_space_bounds.setHigh(1, target_point.y() + tolerance);
        r3_space_bounds.setLow(2, target_point.z() - tolerance);
        r3_space_bounds.setHigh(2, target_point.z() + tolerance);
        r3_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(r3_space_bounds);
        r3_space_information_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(r3_space_));
        r3_space_information_->setup();
        so3_space_ = ompl::base::StateSpacePtr(new ompl::base::PatchedSO3StateSpace());
        so3_space_information_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(so3_space_));
        so3_space_information_->setup();
        // Second, we make samplers for each
        r3_sampler_ = r3_space_information_->allocValidStateSampler();
        so3_sampler_ = so3_space_information_->allocValidStateSampler();
    }

    Eigen::Vector3d sampleR3Point() const
    {
        ompl::base::State* random = r3_space_information_->allocState();
        r3_sampler_->sample(random);
        ompl::base::RealVectorStateSpace::StateType* random_r3 = random->as<ompl::base::RealVectorStateSpace::StateType>();
        Eigen::Vector3d random_state(random_r3->values[0], random_r3->values[1], random_r3->values[2]);
        r3_space_information_->freeState(random);
        return random_state;
    }

    Eigen::Vector3d sampleValidR3Point() const
    {
        while (true)
        {
            Eigen::Vector3d sampled = sampleR3Point();
            double radius = (sampled - target_point_).norm();
            if (radius <= tolerance_)
            {
                return sampled;
            }
        }
    }

    Eigen::Quaterniond sampleSO3Rotation() const
    {
        ompl::base::PatchedSO3StateSpace::StateType random_so3;
        so3_sampler_->sample(&random_so3);
        Eigen::Quaterniond random_rotation(random_so3.w, random_so3.x, random_so3.y, random_so3.z);
        return random_rotation.normalized();
    }

    Eigen::Affine3d sampleInitialGoal() const
    {
        // Sample a point from the tolerance-radius sphere centered around the target point
        Eigen::Vector3d sampled_probe_tip_position = sampleValidR3Point();
        Eigen::Translation3d sampled_probe_tip_translation(sampled_probe_tip_position.x(), sampled_probe_tip_position.y(), sampled_probe_tip_position.z());
        // Sample a rotation in SO(3)
        Eigen::Quaterniond sampled_probe_tip_rotation = sampleSO3Rotation();
        // Combine point and rotation
        Eigen::Affine3d sampled_probe_tip_transform = sampled_probe_tip_translation * sampled_probe_tip_rotation;
        // Backtrack to the probe center
        Eigen::Translation3d inv_probe_tip_translation(0.0, 0.0, -PROBE_LENGTH * 0.5);
        Eigen::Quaterniond inv_probe_tip_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d inv_probe_tip_transform = inv_probe_tip_translation * inv_probe_tip_rotation;
        Eigen::Affine3d sampled_probe_transform = sampled_probe_tip_transform * inv_probe_tip_transform;
        return sampled_probe_transform;
    }

    Eigen::Affine3d sampleValidInitialGoal() const
    {
        // Generate potential goals until a valid goal is generated
        while (true)
        {
            // Sample an initial probe transform
            Eigen::Affine3d potential_probe_transform = sampleInitialGoal();
            // Check if the probe state against the collision planes
            bool state_ok = true;
            // For every plane, make sure both tip and tail of the probe have positive dot product with the plane normal
            // Make the tip offset
            Eigen::Vector3d probe_tip_offset(0.0, 0.0, PROBE_LENGTH * 0.5);
            // Get the probe tip position
            Eigen::Vector3d probe_tip_position = potential_probe_transform * probe_tip_offset;
            // Make the tip offset
            Eigen::Vector3d probe_tail_offset(0.0, 0.0, -PROBE_LENGTH * 0.5);
            // Get the probe tip position
            Eigen::Vector3d probe_tail_position = potential_probe_transform * probe_tail_offset;
            for (size_t idx = 0; idx < collision_planes_.size(); idx++)
            {
                // Get the plane
                const std::pair<Eigen::Vector3d, Eigen::Vector3d>& current_plane = collision_planes_[idx];
                Eigen::Vector3d plane_point = current_plane.first;
                Eigen::Vector3d plane_normal = current_plane.second;
                // Get the vectors
                Eigen::Vector3d tip_vector = probe_tip_position - plane_point;
                Eigen::Vector3d tail_vector = probe_tail_position - plane_point;
                // Get the dot products
                double tip_dot = tip_vector.dot(plane_normal);
                double tail_dot = tail_vector.dot(plane_normal);
                // Check for collision
                if (tip_dot < -0.0 || tail_dot < -0.0)
                {
                    state_ok = false;
                    break;
                }
            }
            if (state_ok)
            {
                return potential_probe_transform;
            }
        }
    }

    virtual void sampleGoal(ompl::base::State* state) const
    {
        // Sample an initial probe transform
        Eigen::Affine3d potential_probe_transform = sampleValidInitialGoal();
        // Extract the SE(3) coordinates and pack them into the state
        state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenTransform(potential_probe_transform);
    }

    inline Eigen::Vector3d GetTipPosition(const deformable_ompl::ProbeStateSpace::StateType* probe_state) const
    {
        // Get the transform of the probe
        Eigen::Affine3d probe_transform = probe_state->getEigenTransform();
        // Make the tip offset
        Eigen::Vector3d probe_tip_offset(0.0, 0.0, PROBE_LENGTH * 0.5);
        // Get the probe tip position
        Eigen::Vector3d probe_tip_position = probe_transform * probe_tip_offset;
        return probe_tip_position;
    }

    inline Eigen::Vector3d GetGoalVector(const deformable_ompl::ProbeStateSpace::StateType* probe_state) const
    {
        // Get the probe tip position
        Eigen::Vector3d probe_tip_position = GetTipPosition(probe_state);
        // Check the R^3 Euclidean distance to the target point
        Eigen::Vector3d error_vector = probe_tip_position - target_point_;
        return error_vector;
    }

    virtual double distanceGoal(const ompl::base::State* state) const
    {
        // Convert the state to our state type
        const deformable_ompl::ProbeStateSpace::StateType* probe_state = state->as<deformable_ompl::ProbeStateSpace::StateType>();
        Eigen::Vector3d error_vector = GetGoalVector(probe_state);
        double error = error_vector.norm();
        // Return the distance to the goal state
        if (error <= tolerance_)
        {
            return 0.0;
        }
        else
        {
            return error - tolerance_;
        }
    }

    virtual unsigned int maxSampleCount() const
    {
        return UINT_MAX;
    }
};

/*
 * Export a solution path to Rviz
 */
void DrawSolutionPath(ompl::base::PlannerStatus& status, std::vector<deformable_ompl::PathState>& path, std::vector<std::pair<Eigen::Vector3d, DVXL>>& probe_points, std::string ns_suffix="")
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
            deformable_ompl::PathState& probe_state = path[idx];
            geometry_msgs::Vector3 position = probe_state.position;
            geometry_msgs::Quaternion orientation = probe_state.orientation;
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
            state_rep.pose.position.x = position.x;
            state_rep.pose.position.y = position.y;
            state_rep.pose.position.z = position.z;
            state_rep.pose.orientation.w = orientation.w;
            state_rep.pose.orientation.x = orientation.x;
            state_rep.pose.orientation.y = orientation.y;
            state_rep.pose.orientation.z = orientation.z;
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
 * Compute the transform with Z axis pointing along the plane normal
 */
Eigen::Affine3d ComputePlaneDisplayTransform(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& plane)
{
    const Eigen::Vector3d& plane_point = plane.first;
    const Eigen::Vector3d& plane_normal = plane.second;
    Eigen::Translation3d plane_display_translation(plane_point.x(), plane_point.y(), plane_point.z());
    // Compute the pointing rotation using the cross product
    Eigen::Vector3d base_normalized_vector(0.0, 0.0, 1.0);
    Eigen::Vector3d cross_product = base_normalized_vector.cross(plane_normal);
    double dot_product = base_normalized_vector.dot(plane_normal);
    double angle = acos(dot_product);
    // Make the rotation
    Eigen::AngleAxisd raw_rotation(angle, cross_product);
    Eigen::Quaterniond plane_display_rotation(raw_rotation);
    // Assemble the transform
    Eigen::Affine3d plane_display_transform = plane_display_translation * plane_display_rotation;
    return plane_display_transform;
}

/*
 * Draw collision planes (if any)
 */
void DrawCollisionPlanes(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& collision_planes)
{
    if (collision_planes.size() > 0)
    {
        double env_size = std::max({g_environment.GetXSize(), g_environment.GetYSize(), g_environment.GetZSize()});
        visualization_msgs::MarkerArray planes_rep;
        // Loop through the path
        for (size_t idx = 0; idx < collision_planes.size(); idx++)
        {
            const std::pair<Eigen::Vector3d, Eigen::Vector3d>& current_plane = collision_planes[idx];
            Eigen::Affine3d plane_display_transform = ComputePlaneDisplayTransform(current_plane);
            Eigen::Vector3d plane_display_translation = plane_display_transform.translation();
            Eigen::Quaterniond plane_display_rotation(plane_display_transform.rotation());
            // Make the display rep for the state
            visualization_msgs::Marker plane_rep;
            // Populate the header
            plane_rep.header.frame_id = "deformable_ompl";
            // Populate the options
            plane_rep.ns = "collision_planes";
            plane_rep.id = idx + 1;
            plane_rep.type = visualization_msgs::Marker::CUBE;
            plane_rep.action = visualization_msgs::Marker::ADD;
            plane_rep.lifetime = ros::Duration(0.0);
            plane_rep.frame_locked = false;
            plane_rep.scale.x = env_size;
            plane_rep.scale.y = env_size;
            plane_rep.scale.z = 0.05;
            plane_rep.pose.position.x = plane_display_translation.x();
            plane_rep.pose.position.y = plane_display_translation.y();
            plane_rep.pose.position.z = plane_display_translation.z();
            plane_rep.pose.orientation.w = plane_display_rotation.w();
            plane_rep.pose.orientation.x = plane_display_rotation.x();
            plane_rep.pose.orientation.y = plane_display_rotation.y();
            plane_rep.pose.orientation.z = plane_display_rotation.z();
            plane_rep.color.r = 0.25;
            plane_rep.color.b = 0.25;
            plane_rep.color.g = 0.0;
            plane_rep.color.a = 1.0;
            planes_rep.markers.push_back(plane_rep);
        }
        // Publish
        ROS_INFO("Drawing %zu collision planes", planes_rep.markers.size());
        g_debug_marker_pub.publish(planes_rep);
    }
    else
    {
        ROS_INFO("No collision planes to draw");
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
 * Function to sample n possible goal states, and then select the lowest cost goal state
 */
Eigen::Affine3d SampleGoalState(const ompl::base::GoalPtr& goal_region, const deformable_ompl::ProbeDVXLCostValidityChecker& dvxl_checker, const u_int32_t num_presampled_goals)
{
    ROS_INFO("Sampling %u potential goals...", num_presampled_goals);
    // Sample n goals
    std::vector<std::pair<double, Eigen::Affine3d>> sampled_goals(num_presampled_goals);
    for (u_int32_t idx = 0; idx < num_presampled_goals; idx++)
    {
        Eigen::Affine3d sampled_goal = goal_region->as<SampleableProbeGoalRegion>()->sampleValidInitialGoal();
        double sampled_goal_cost = dvxl_checker.CheckStateTransformCost(sampled_goal);
        sampled_goals[idx].first = sampled_goal_cost;
        sampled_goals[idx].second = sampled_goal;
    }
    // Pick the best goal
    double best_cost = INFINITY;
    Eigen::Affine3d best_sampled_goal;
    for (size_t idx = 0; idx < sampled_goals.size(); idx++)
    {
        if (sampled_goals[idx].first < best_cost)
        {
            best_cost = sampled_goals[idx].first;
            best_sampled_goal = sampled_goals[idx].second;
        }
    }
    ROS_INFO("...sampled best goal with cost %f", best_cost);
    Eigen::Vector3d best_sampled_goal_translation = best_sampled_goal.translation();
    Eigen::Quaterniond best_sampled_goal_rotation(best_sampled_goal.rotation());
    ROS_INFO("Sampled goal with transform (x,y,z)(x,y,z,w) (%f,%f,%f)(%f,%f,%f,%f)", best_sampled_goal_translation.x(), best_sampled_goal_translation.y(), best_sampled_goal_translation.z(), best_sampled_goal_rotation.x(), best_sampled_goal_rotation.y(), best_sampled_goal_rotation.z(), best_sampled_goal_rotation.w());
    // Return the goal
    return best_sampled_goal;
}

/*
 * Service callback that plans a low-cost path using RRT* between the provided start config and goal region in the provided environment
 */
bool ProbePlanPathServiceCB(deformable_ompl::PlanPath::Request& req, deformable_ompl::PlanPath::Response& res)
{
    ROS_INFO("Processing PlanPath service...");
    g_environment = RebuildEnvironment(req.query.obstacles, COST_DVXL_RESOLUTION, true);
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
    ompl::base::StateSpacePtr space(new ProbeStateSpace());
    // Set the bounds of the space
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(0, PROBE_MOUNT_RADIUS);
    bounds.setLow(1, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(1, PROBE_MOUNT_RADIUS);
    bounds.setLow(2, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(2, PROBE_MOUNT_RADIUS);
    space->as<ProbeStateSpace>()->setR3Bounds(bounds);
    ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));
    // Set up the collision checker
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> collision_planes;
    for (size_t idx = 0; idx < req.query.collision_planes.size(); idx++)
    {
        geometry_msgs::Point collision_plane_point = req.query.collision_planes[idx].point;
        geometry_msgs::Vector3 collision_plane_normal = req.query.collision_planes[idx].normal;
        Eigen::Vector3d plane_point(collision_plane_point.x, collision_plane_point.y, collision_plane_point.z);
        Eigen::Vector3d plane_normal(collision_plane_normal.x, collision_plane_normal.y, collision_plane_normal.z);
        std::pair<Eigen::Vector3d, Eigen::Vector3d> collision_plane(plane_point, plane_normal);
        collision_planes.push_back(collision_plane);
    }
    DrawCollisionPlanes(collision_planes);
    ProbeDVXLCostValidityChecker dvxl_checker(space_information, g_environment, puncture_environment, req.query.probe_radius, collision_planes);
    //dvxl_checker.setVerbosity(true);
    std::vector<std::pair<Eigen::Vector3d, DVXL>> probe_points = dvxl_checker.GetProbePoints();
    ompl::base::StateValidityCheckerPtr dvxl_validity_checker(&dvxl_checker, dealocate_StateValidityChecker_fn);
    space_information->setStateValidityChecker(dvxl_validity_checker);
    // Set up the motion validity checker (this lets us check for punctures)
    ompl::base::MotionValidatorPtr dvxl_motion_validator(&dvxl_checker, dealocate_MotionValidator_fn);
    space_information->setMotionValidator(dvxl_motion_validator);
    // Finish setup of the space
    space_information->setup();
    // Make the start state
    Eigen::Vector3d start_position(req.query.start.position.x, req.query.start.position.y, req.query.start.position.z);
    Eigen::Quaterniond start_orientation(req.query.start.orientation.w, req.query.start.orientation.x, req.query.start.orientation.y, req.query.start.orientation.z);
    ompl::base::ScopedState<> start(space);
    start->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenPosition(start_position);
    start->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenRotation(start_orientation);
    // Create the problem definition
    ompl::base::ProblemDefinitionPtr problem_definition(new ompl::base::ProblemDefinition(space_information));
    // Set the start and goal states
    problem_definition->addStartState(start);
    // Set the goal type
    if (req.query.goal_type == deformable_ompl::PlanPathQuery::GOAL_REGION)
    {
        // Make the goal region
        Eigen::Vector3d target_point(req.query.target.x, req.query.target.y, req.query.target.z);
        ompl::base::GoalPtr goal(new SampleableProbeGoalRegion(space_information, collision_planes, target_point, req.query.target_tolerance));
        problem_definition->setGoal(goal);
    }
    else if (req.query.goal_type == deformable_ompl::PlanPathQuery::PRESAMPLE_GOAL_STATE)
    {
        // Make the goal region
        Eigen::Vector3d target_point(req.query.target.x, req.query.target.y, req.query.target.z);
        ompl::base::GoalPtr goal_region(new SampleableProbeGoalRegion(space_information, collision_planes, target_point, req.query.target_tolerance));
        // Sample a goal state
        ompl::base::ScopedState<> goal(space);
        Eigen::Affine3d sampled_goal_transform = SampleGoalState(goal_region, dvxl_checker, req.query.presampled_goals);
        goal->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenTransform(sampled_goal_transform);
        // Set the goal state
        problem_definition->setGoalState(goal);
    }
    else if (req.query.goal_type == deformable_ompl::PlanPathQuery::GOAL_STATE)
    {
        // Make the goal state
        Eigen::Vector3d goal_position(req.query.goal.position.x, req.query.goal.position.y, req.query.goal.position.z);
        Eigen::Quaterniond goal_orientation(req.query.goal.orientation.w, req.query.goal.orientation.x, req.query.goal.orientation.y, req.query.goal.orientation.z);
        ompl::base::ScopedState<> goal(space);
        goal->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenPosition(goal_position);
        goal->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenRotation(goal_orientation);
        problem_definition->setGoalState(goal);
    }
    else
    {
        res.result.status = deformable_ompl::PlanPathResult::INVALID_PARAMETERS;
        return true;
    }
    // Setup the cost function
    if (req.query.p < 0.0 || req.query.p > 1.0)
    {
        res.result.status = deformable_ompl::PlanPathResult::INVALID_PARAMETERS;
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
    if (req.query.planner_type == deformable_ompl::PlanPathQuery::RRTSTAR)
    {
        planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(space_information));
    }
    else if (req.query.planner_type == deformable_ompl::PlanPathQuery::TRRT)
    {
        planner = ompl::base::PlannerPtr(new ompl::geometric::TRRT(space_information));
    }
    else if (req.query.planner_type == deformable_ompl::PlanPathQuery::PRMSTAR)
    {
        planner = ompl::base::PlannerPtr(new ompl::geometric::PRMstar(space_information));
    }
    else
    {
        res.result.status = deformable_ompl::PlanPathResult::INVALID_PARAMETERS;
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
        deformable_ompl::ProbeStateSpace::StateType* first_probe_state = states[0]->as<deformable_ompl::ProbeStateSpace::StateType>();
        Eigen::Vector3d first_state_position = first_probe_state->getEigenPosition();
        Eigen::Quaterniond first_state_orientation = first_probe_state->getEigenRotation();
        deformable_ompl::PathState first_path_state;
        first_path_state.position.x = first_state_position.x();
        first_path_state.position.y = first_state_position.y();
        first_path_state.position.z = first_state_position.z();
        first_path_state.orientation.w = first_state_orientation.w();
        first_path_state.orientation.x = first_state_orientation.x();
        first_path_state.orientation.y = first_state_orientation.y();
        first_path_state.orientation.z = first_state_orientation.z();
        first_path_state.deformation_cost = dvxl_cost_fn->stateCost(first_probe_state).GET_OMPL_COST;
        first_path_state.length_cost = path_length_cost_fn->stateCost(first_probe_state).GET_OMPL_COST;
        res.result.path.push_back(first_path_state);
        // Add the following states
        for (size_t idx = 1; idx < states.size(); idx++)
        {
            // Get the start and end states of this section of the path
            deformable_ompl::ProbeStateSpace::StateType* start_probe_state = states[idx - 1]->as<deformable_ompl::ProbeStateSpace::StateType>();
            deformable_ompl::ProbeStateSpace::StateType* end_probe_state = states[idx]->as<deformable_ompl::ProbeStateSpace::StateType>();
            // We want to add all the intermediate states to our returned path
            unsigned int num_segments = space_information->getStateSpace()->validSegmentCount(start_probe_state, end_probe_state);
            // If there's only one segment, we just add the end state of the window
            if (num_segments <= 1)
            {
                Eigen::Vector3d position = end_probe_state->getEigenPosition();
                Eigen::Quaterniond orientation = end_probe_state->getEigenRotation();
                deformable_ompl::PathState path_state;
                path_state.position.x = position.x();
                path_state.position.y = position.y();
                path_state.position.z = position.z();
                path_state.orientation.w = orientation.w();
                path_state.orientation.x = orientation.x();
                path_state.orientation.y = orientation.y();
                path_state.orientation.z = orientation.z();
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
                    Eigen::Vector3d position = intermediate_state->as<deformable_ompl::ProbeStateSpace::StateType>()->getEigenPosition();
                    Eigen::Quaterniond orientation = intermediate_state->as<deformable_ompl::ProbeStateSpace::StateType>()->getEigenRotation();
                    deformable_ompl::PathState path_state;
                    path_state.position.x = position.x();
                    path_state.position.y = position.y();
                    path_state.position.z = position.z();
                    path_state.orientation.w = orientation.w();
                    path_state.orientation.x = orientation.x();
                    path_state.orientation.y = orientation.y();
                    path_state.orientation.z = orientation.z();
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
        // Compute the error
        // Get the goal
        if (req.query.goal_type == deformable_ompl::PlanPathQuery::GOAL_REGION)
        {
            const ompl::base::GoalPtr& goal = problem_definition->getGoal();
            deformable_ompl::ProbeStateSpace::StateType* end_state = states[states.size() - 1]->as<deformable_ompl::ProbeStateSpace::StateType>();
            Eigen::Vector3d tip_position = goal->as<SampleableProbeGoalRegion>()->GetTipPosition(end_state);
            res.result.tip_position.x = tip_position.x();
            res.result.tip_position.y = tip_position.y();
            res.result.tip_position.z = tip_position.z();
            Eigen::Vector3d error = goal->as<SampleableProbeGoalRegion>()->GetGoalVector(end_state);
            res.result.error.x = error.x();
            res.result.error.y = error.y();
            res.result.error.z = error.z();
        }
        else
        {
            res.result.tip_position.x = 0.0;
            res.result.tip_position.y = 0.0;
            res.result.tip_position.z = 0.0;
            res.result.error.x = 0.0;
            res.result.error.y = 0.0;
            res.result.error.z = 0.0;
        }
        // Set the runtime, cost, and length
        double secs = (double)(et.tv_sec - st.tv_sec);
        secs = secs + ((double)(et.tv_nsec - st.tv_nsec) / 1000000000.0);
        res.result.run_time = secs;
        res.result.total_cost = path->cost(cost_fn).GET_OMPL_COST;
        res.result.total_length = path->length();
        // Set the status and return
        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            res.result.status = deformable_ompl::PlanPathResult::SUCCESS;
            ROS_INFO("...returning successful plan");
        }
        else
        {
            res.result.status = deformable_ompl::PlanPathResult::APPROXIMATE;
            ROS_ERROR("...returning unsuccessful approximate plan");
        }
        return true;
    }
    else if (status == ompl::base::PlannerStatus::INVALID_START)
    {
        // Return invalid start error status
        res.result.status = deformable_ompl::PlanPathResult::INVALID_START;
        ROS_ERROR("...invalid start");
        return true;
    }
    else if (status == ompl::base::PlannerStatus::INVALID_GOAL)
    {
        // Return invalid goal error status
        res.result.status = deformable_ompl::PlanPathResult::INVALID_GOAL;
        ROS_ERROR("...invalid goal");
        return true;
    }
    else if (status == ompl::base::PlannerStatus::TIMEOUT)
    {
        // Return timeout error status
        res.result.status = deformable_ompl::PlanPathResult::TIMEOUT;
        ROS_ERROR("...timeout");
        return true;
    }
    else
    {
        // Return unknown error status
        res.result.status = deformable_ompl::PlanPathResult::UNKNOWN_ERROR;
        ROS_ERROR("...unknown error");
        return true;
    }
}

bool ComputeDVXLCostServiceCB(deformable_ompl::ComputeDVXLCost::Request& req, deformable_ompl::ComputeDVXLCost::Response& res)
{
    ROS_INFO("Processing ComputeDVXLCost service...");
    ROS_INFO("Building evironment...");
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
    ompl::base::StateSpacePtr space(new ProbeStateSpace());
    // Set the bounds of the space
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(0, PROBE_MOUNT_RADIUS);
    bounds.setLow(1, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(1, PROBE_MOUNT_RADIUS);
    bounds.setLow(2, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(2, PROBE_MOUNT_RADIUS);
    space->as<ProbeStateSpace>()->setR3Bounds(bounds);
    ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));
    // Set up the collision checker
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> collision_planes;
    ProbeDVXLCostValidityChecker dvxl_checker(space_information, g_environment, puncture_environment, req.query.probe_radius, collision_planes);
    dvxl_checker.setVerbosity(true);
    std::vector<std::pair<Eigen::Vector3d, DVXL>> probe_points = dvxl_checker.GetProbePoints();
    ROS_INFO("Drawing path in RVIZ...");
    ompl::base::PlannerStatus dummy_status = ompl::base::PlannerStatus::EXACT_SOLUTION;
    DrawSolutionPath(dummy_status, req.query.path, probe_points);
    ROS_INFO("Checking path for punctures...");
    bool validity_check = dvxl_checker.IncrementalCheckFinalPath(req.query.path, g_debug_marker_pub);
    if (validity_check)
    {
        ROS_INFO("Passed validity and puncture check");
        ROS_INFO("Computing path DVXL cost...");
        double path_dvxl_cost = dvxl_checker.ComputeCostFinalPath(req.query.path, g_debug_marker_pub);
        ROS_INFO("Computed DVXL cost %f...", path_dvxl_cost);
        deformable_ompl::FeatureValues feature_val;
        feature_val.feature_name = "total";
        feature_val.feature_values.push_back(path_dvxl_cost);
        feature_val.cumulative_feature_value = path_dvxl_cost;
        res.result.features.push_back(feature_val);
        return true;
    }
    else
    {
        ROS_WARN("Failed validity and puncture check - returning infinite cost");
        deformable_ompl::FeatureValues feature_val;
        feature_val.feature_name = "total";
        feature_val.feature_values.push_back(INFINITY);
        feature_val.cumulative_feature_value = INFINITY;
        res.result.features.push_back(feature_val);
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

bool ResamplePathServiceCB(deformable_ompl::ResamplePath::Request& req, deformable_ompl::ResamplePath::Response& res)
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
    ompl::base::StateSpacePtr space(new ProbeStateSpace());
    // Set the bounds of the space
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(0, PROBE_MOUNT_RADIUS);
    bounds.setLow(1, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(1, PROBE_MOUNT_RADIUS);
    bounds.setLow(2, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(2, PROBE_MOUNT_RADIUS);
    space->as<ProbeStateSpace>()->setR3Bounds(bounds);
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
        const deformable_ompl::PathState& prev_state = req.query.original_path[idx - 1];
        const deformable_ompl::PathState& curr_state = req.query.original_path[idx];
        // Convert them to OMPL states
        ompl::base::State* prev_probe_state = space_information->allocState();
        prev_probe_state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenPosition(Eigen::Vector3d(prev_state.position.x, prev_state.position.y, prev_state.position.z));
        prev_probe_state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenRotation(Eigen::Quaterniond(prev_state.orientation.w, prev_state.orientation.x, prev_state.orientation.y, prev_state.orientation.z));
        ompl::base::State* curr_probe_state = space_information->allocState();
        curr_probe_state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenPosition(Eigen::Vector3d(curr_state.position.x, curr_state.position.y, curr_state.position.z));
        curr_probe_state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenRotation(Eigen::Quaterniond(curr_state.orientation.w, curr_state.orientation.x, curr_state.orientation.y, curr_state.orientation.z));
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
                Eigen::Vector3d position = intermediate_state->as<deformable_ompl::ProbeStateSpace::StateType>()->getEigenPosition();
                Eigen::Quaterniond orientation = intermediate_state->as<deformable_ompl::ProbeStateSpace::StateType>()->getEigenRotation();
                deformable_ompl::PathState path_state;
                path_state.position.x = position.x();
                path_state.position.y = position.y();
                path_state.position.z = position.z();
                path_state.orientation.w = orientation.w();
                path_state.orientation.x = orientation.x();
                path_state.orientation.y = orientation.y();
                path_state.orientation.z = orientation.z();
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

std::vector<deformable_ompl::PathState> ShortcutPath(const std::vector<deformable_ompl::PathState>& original_path, const double max_shortcut_fraction, const ompl::base::SpaceInformationPtr& space_information, ompl::RNG& prng)
{
    std::vector<deformable_ompl::PathState> shortcut_path;
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
    const deformable_ompl::PathState& start_state = original_path[start_index];
    const deformable_ompl::PathState& end_state = original_path[end_index];
    // Convert them to OMPL states
    ompl::base::State* start_probe_state = space_information->allocState();
    start_probe_state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenPosition(Eigen::Vector3d(start_state.position.x, start_state.position.y, start_state.position.z));
    start_probe_state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenRotation(Eigen::Quaterniond(start_state.orientation.w, start_state.orientation.x, start_state.orientation.y, start_state.orientation.z));
    ompl::base::State* end_probe_state = space_information->allocState();
    end_probe_state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenPosition(Eigen::Vector3d(end_state.position.x, end_state.position.y, end_state.position.z));
    end_probe_state->as<deformable_ompl::ProbeStateSpace::StateType>()->setEigenRotation(Eigen::Quaterniond(end_state.orientation.w, end_state.orientation.x, end_state.orientation.y, end_state.orientation.z));
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
            Eigen::Vector3d position = intermediate_state->as<deformable_ompl::ProbeStateSpace::StateType>()->getEigenPosition();
            Eigen::Quaterniond orientation = intermediate_state->as<deformable_ompl::ProbeStateSpace::StateType>()->getEigenRotation();
            deformable_ompl::PathState path_state;
            path_state.position.x = position.x();
            path_state.position.y = position.y();
            path_state.position.z = position.z();
            path_state.orientation.w = orientation.w();
            path_state.orientation.x = orientation.x();
            path_state.orientation.y = orientation.y();
            path_state.orientation.z = orientation.z();
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

bool SimplifyPathServiceCB(deformable_ompl::SimplifyPath::Request& req, deformable_ompl::SimplifyPath::Response& res)
{
    ROS_INFO("Processing SimplifyPath service...");
    if (req.query.max_shortcut_fraction < 0.0 || req.query.max_shortcut_fraction > 1.0)
    {
        ROS_ERROR("Invalid max_shortcut_fraction - it must be between 0 and 1");
        res.result.status = deformable_ompl::SimplifyPathResult::INVALID_PARAMETERS;
        return true;
    }
    if (req.query.original_path.size() < 3)
    {
        ROS_INFO("Path is already as simple as possible - no simplification will be performed");
        res.result.simplified_path = req.query.original_path;
        res.result.status = deformable_ompl::SimplifyPathResult::SUCCESS;
        return true;
    }
    g_environment = RebuildEnvironment(req.query.obstacles, COST_DVXL_RESOLUTION, true);
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
    ompl::base::StateSpacePtr space(new ProbeStateSpace());
    // Set the bounds of the space
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(0, PROBE_MOUNT_RADIUS);
    bounds.setLow(1, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(1, PROBE_MOUNT_RADIUS);
    bounds.setLow(2, -PROBE_MOUNT_RADIUS);
    bounds.setHigh(2, PROBE_MOUNT_RADIUS);
    space->as<ProbeStateSpace>()->setR3Bounds(bounds);
    ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));
    // Set up the collision checker
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> collision_planes;
    for (size_t idx = 0; idx < req.query.collision_planes.size(); idx++)
    {
        geometry_msgs::Point collision_plane_point = req.query.collision_planes[idx].point;
        geometry_msgs::Vector3 collision_plane_normal = req.query.collision_planes[idx].normal;
        Eigen::Vector3d plane_point(collision_plane_point.x, collision_plane_point.y, collision_plane_point.z);
        Eigen::Vector3d plane_normal(collision_plane_normal.x, collision_plane_normal.y, collision_plane_normal.z);
        std::pair<Eigen::Vector3d, Eigen::Vector3d> collision_plane(plane_point, plane_normal);
        collision_planes.push_back(collision_plane);
    }
    DrawCollisionPlanes(collision_planes);
    ProbeDVXLCostValidityChecker dvxl_checker(space_information, g_environment, puncture_environment, req.query.probe_radius, collision_planes);
    // Make a RNG
    ompl::RNG prng;
    ROS_INFO("Setup complete, simplifing for up to %u iterations (or until %u failed simplifications)", req.query.max_iterations, req.query.max_failed_iterations);
    struct timespec st, et;
    clock_gettime(CLOCK_MONOTONIC, &st);
    // Resample the original path to make it denser
    deformable_ompl::ResamplePath::Request resample_req;
    resample_req.query.original_path = req.query.original_path;
    resample_req.query.interval_size = 0.075;
    deformable_ompl::ResamplePath::Response resample_res;
    ResamplePathServiceCB(resample_req, resample_res);
    // Compute stats
    double original_length = dvxl_checker.ComputePathLength(resample_res.result.resampled_path);
    double original_dvxl_cost = dvxl_checker.ComputeCostFinalPath(resample_res.result.resampled_path, g_debug_marker_pub);
    // Store state
    double current_length = original_length;
    double current_dvxl_cost = original_dvxl_cost;
    std::vector<deformable_ompl::PathState> current_path = resample_res.result.resampled_path;
    u_int32_t num_iterations = 0;
    u_int32_t failed_iterations = 0;
    while (num_iterations < req.query.max_iterations && failed_iterations < req.query.max_failed_iterations && current_path.size() > 2)
    {
        std::cout << "Shortcut iteration " << num_iterations << std::endl;
        // Shortcut the current path
        std::vector<deformable_ompl::PathState> raw_shortcut_path = ShortcutPath(current_path, req.query.max_shortcut_fraction, space_information, prng);
        deformable_ompl::ResamplePath::Request resample_shortcut_req;
        resample_shortcut_req.query.original_path = raw_shortcut_path;
        resample_shortcut_req.query.interval_size = 0.075;
        deformable_ompl::ResamplePath::Response resample_shortcut_res;
        ResamplePathServiceCB(resample_shortcut_req, resample_shortcut_res);
        std::vector<deformable_ompl::PathState> shortcut_path = resample_shortcut_res.result.resampled_path;
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
    res.result.status = deformable_ompl::SimplifyPathResult::SUCCESS;
    ROS_INFO("Drawing path in RVIZ...");
    ompl::base::PlannerStatus dummy_status = ompl::base::PlannerStatus::EXACT_SOLUTION;
    std::vector<std::pair<Eigen::Vector3d, DVXL>> probe_points = dvxl_checker.GetProbePoints();
    DrawSolutionPath(dummy_status, res.result.simplified_path, probe_points, "simplified");
    return true;
}

visualization_msgs::Marker ExportWatershedMapForDisplay(const VoxelGrid::VoxelGrid<Eigen::Vector3d>& watershed_map, const std::string& frame, const std_msgs::ColorRGBA& inside_color, const std_msgs::ColorRGBA& outside_color)
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame;
    // Populate the options
    display_rep.ns = "watershed_map_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = watershed_map.GetCellSizes()[0];
    display_rep.scale.y = watershed_map.GetCellSizes()[1];
    display_rep.scale.z = watershed_map.GetCellSizes()[2];
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < watershed_map.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < watershed_map.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < watershed_map.GetNumZCells(); z_index++)
            {
                // Convert watershed map indices into a real-world location
                std::vector<double> location = watershed_map.GridIndexToLocation(x_index, y_index, z_index);
                geometry_msgs::Point new_point;
                new_point.x = location[0];
                new_point.y = location[1];
                new_point.z = location[2];
                display_rep.points.push_back(new_point);
                // Check if the point is inside or outside
                const Eigen::Vector3d& local_maxima = watershed_map.GetImmutable(x_index, y_index, z_index).first;
                if (local_maxima.x() == INFINITY || local_maxima.y() == INFINITY || local_maxima.z() == INFINITY)
                {
                    display_rep.colors.push_back(outside_color);
                }
                else
                {
                    display_rep.colors.push_back(inside_color);
                }
            }
        }
    }
    return display_rep;
}

Eigen::Vector3d GenerateRandomCandidateTarget(const Eigen::Vector3d& edge_point, const double min_radius, const double max_radius, ompl::RNG& prng)
{
    while (true)
    {
        double candidate_delta_x = prng.uniformReal(-max_radius, max_radius);
        double candidate_delta_y = prng.uniformReal(-max_radius, max_radius);
        double candidate_delta_z = prng.uniformReal(-max_radius, max_radius);
        double distance = sqrt(pow(candidate_delta_x, 2.0) + pow(candidate_delta_y, 2.0) + pow(candidate_delta_z, 2.0));
        if (distance >= min_radius && distance <= max_radius)
        {
            Eigen::Vector3d candidate_target(edge_point.x() + candidate_delta_x, edge_point.y() + candidate_delta_y, edge_point.z() + candidate_delta_z);
            return candidate_target;
        }
    }
}

bool ComputeTrialSetupServiceCB(deformable_ompl::ComputeTrialSetup::Request& req, deformable_ompl::ComputeTrialSetup::Response& res)
{
    ROS_INFO("Processing ComputeTrialSetup service...");
    ROS_INFO("Target object 1: %u Target object 2: %u", req.query.first_object_id, req.query.second_object_id);
    if (req.query.first_object_id == req.query.second_object_id)
    {
        ROS_ERROR("Objects must be different");
        res.result.status = deformable_ompl::ComputeTrialSetupResult::INVALID_PARAMETERS;
        return true;
    }
    // Retrieve the two object center points
    Eigen::Vector3d first_object_center_point(0.0, 0.0, 0.0);
    Eigen::Vector3d second_object_center_point(0.0, 0.0, 0.0);
    int32_t first_object_found = 0;
    int32_t second_object_found = 0;
    for (size_t idx = 0; idx < req.query.obstacles.size(); idx++)
    {
        const deformable_ompl::ObstacleConfig& current_object = req.query.obstacles[idx];
        if (current_object.obstacle_id == req.query.first_object_id)
        {
            first_object_center_point = Eigen::Vector3d(current_object.obstacle_pose.position.x, current_object.obstacle_pose.position.y, current_object.obstacle_pose.position.z);
            first_object_found++;
        }
        if (current_object.obstacle_id == req.query.second_object_id)
        {
            second_object_center_point = Eigen::Vector3d(current_object.obstacle_pose.position.x, current_object.obstacle_pose.position.y, current_object.obstacle_pose.position.z);
            second_object_found++;
        }
    }
    if (first_object_found < 1 || second_object_found < 1)
    {
        ROS_ERROR("One or both objects do not exist in the environment");
        res.result.status = deformable_ompl::ComputeTrialSetupResult::INVALID_PARAMETERS;
        return true;
    }
    else if (first_object_found > 1 || second_object_found > 1)
    {
        ROS_ERROR("One or both objects exist multiple times in the environment");
        res.result.status = deformable_ompl::ComputeTrialSetupResult::INVALID_PARAMETERS;
        return true;
    }
    // Build the environment
    ROS_INFO("Building evironment...");
    g_environment = RebuildEnvironment(req.query.obstacles, COST_DVXL_RESOLUTION, false);
    DVXLCostFn dvxl_cost_fn(g_environment);
    // Generate all possible edge points
    ROS_INFO("Computing the \"minimum Voronoi points\" between objects %u and %u...", req.query.first_object_id, req.query.second_object_id);
    std::vector<Eigen::Vector3d> minimum_voronoi_points = dvxl_cost_fn.ExtractMinimumVoronoiPoints(req.query.first_object_id, req.query.second_object_id);
    if (minimum_voronoi_points.size() == 0)
    {
        ROS_ERROR("No minimum Voronoi points - i.e. no edge points");
        res.result.status = deformable_ompl::ComputeTrialSetupResult::NO_EDGE_POINTS;
        return true;
    }
    // We want to avoid edge points at the very end of an edge
    // Pick two points at random
    ompl::RNG prng;
    u_int32_t random_first_edge_point_index = (u_int32_t)prng.uniformInt(0, minimum_voronoi_points.size() - 1);
    u_int32_t random_second_edge_point_index = (u_int32_t)prng.uniformInt(0, minimum_voronoi_points.size() - 1);
    //u_int32_t edge_point_index = (random_first_edge_point_index / 2) + (random_second_edge_point_index / 2);
    u_int32_t edge_point_index = random_first_edge_point_index;
    ROS_INFO("Ranbom indices: %u, %u, selected %u as edge point index", random_first_edge_point_index, random_second_edge_point_index, edge_point_index);
    Eigen::Vector3d edge_point = minimum_voronoi_points[edge_point_index];
    ROS_INFO("...selected edge point: %s from %zu possibilities", PrettyPrint::PrettyPrint(edge_point).c_str(), minimum_voronoi_points.size());
    // Pick a point on the line between the two objects
    // First, get the full environment SDF
    const sdf_tools::SignedDistanceField& sdf = dvxl_cost_fn.GetSDF();
    // Second, make a "Watershed Map" from the SDF
    ROS_INFO("Generating Watershed Map from SDF...");
    VoxelGrid::VoxelGrid<Eigen::Vector3d> watershed_map = sdf.ComputeLocalMaximaMap();
    ROS_INFO("...generated Watershed Map");
    // Draw our progress so far
    visualization_msgs::MarkerArray env_rep;
    std_msgs::ColorRGBA inside_color;
    inside_color.r = 0.1;
    inside_color.g = 0.1;
    inside_color.b = 0.1;
    inside_color.a = 0.1;
    std_msgs::ColorRGBA outside_color;
    outside_color.r = 0.0;
    outside_color.g = 0.0;
    outside_color.b = 0.0;
    outside_color.a = 0.0;
    visualization_msgs::Marker watershed_map_display_rep = ExportWatershedMapForDisplay(watershed_map, "deformable_ompl", inside_color, outside_color);
    env_rep.markers.push_back(watershed_map_display_rep);
    visualization_msgs::Marker env_marker = g_environment.ExportForDisplay();
    env_rep.markers.push_back(env_marker);
    g_debug_marker_pub.publish(env_rep);
    // Search a circle around the edge point for a point that is:
    // - "inside" the grid
    // - "inside" according to the watershed map
    // - >= probe diameter from an obstacle
    ROS_INFO("Searching for a target point with %u checks...", req.query.maximum_target_checks);
    u_int32_t target_checks = 0;
    while (target_checks < req.query.maximum_target_checks)
    {
        // Pick a random target point around the edge point
        Eigen::Vector3d random_candidate_target = GenerateRandomCandidateTarget(edge_point, req.query.minimum_target_depth, req.query.maximum_target_depth, prng);
        // Check if it meets the conditions
        const Eigen::Vector3d& local_maxima = watershed_map.GetImmutable(random_candidate_target).first;
        std::pair<float, bool> distance_query = sdf.GetSafe(random_candidate_target);
        if (distance_query.second && distance_query.first >= req.query.minimum_target_clearance && local_maxima.x() != INFINITY && local_maxima.y() != INFINITY && local_maxima.z() != INFINITY)
        {
            ROS_INFO("...target point found: %s", PrettyPrint::PrettyPrint(random_candidate_target).c_str());
            res.result.edge_point.x = edge_point.x();
            res.result.edge_point.y = edge_point.y();
            res.result.edge_point.z = edge_point.z();
            res.result.target_point.x = random_candidate_target.x();
            res.result.target_point.y = random_candidate_target.y();
            res.result.target_point.z = random_candidate_target.z();
            // Draw the edge point and target point
            visualization_msgs::Marker edge_point_rep;
            // Populate the header
            edge_point_rep.header.frame_id = "deformable_ompl";
            // Populate the options
            edge_point_rep.ns = "edge_point";
            edge_point_rep.id = 1;
            edge_point_rep.type = visualization_msgs::Marker::SPHERE;
            edge_point_rep.action = visualization_msgs::Marker::ADD;
            edge_point_rep.lifetime = ros::Duration(0.0);
            edge_point_rep.frame_locked = false;
            edge_point_rep.scale.x = 0.5;
            edge_point_rep.scale.y = 0.5;
            edge_point_rep.scale.z = 0.5;
            edge_point_rep.pose.position.x = edge_point.x();
            edge_point_rep.pose.position.y = edge_point.y();
            edge_point_rep.pose.position.z = edge_point.z();
            edge_point_rep.pose.orientation.x = 0.0;
            edge_point_rep.pose.orientation.y = 0.0;
            edge_point_rep.pose.orientation.z = 0.0;
            edge_point_rep.pose.orientation.w = 1.0;
            edge_point_rep.color.r = 0.0;
            edge_point_rep.color.g = 1.0;
            edge_point_rep.color.b = 1.0;
            edge_point_rep.color.a = 1.0;
            visualization_msgs::Marker target_point_rep;
            // Populate the header
            target_point_rep.header.frame_id = "deformable_ompl";
            // Populate the options
            target_point_rep.ns = "target_point";
            target_point_rep.id = 1;
            target_point_rep.type = visualization_msgs::Marker::SPHERE;
            target_point_rep.action = visualization_msgs::Marker::ADD;
            target_point_rep.lifetime = ros::Duration(0.0);
            target_point_rep.frame_locked = false;
            target_point_rep.scale.x = 0.5;
            target_point_rep.scale.y = 0.5;
            target_point_rep.scale.z = 0.5;
            target_point_rep.pose.position.x = random_candidate_target.x();
            target_point_rep.pose.position.y = random_candidate_target.y();
            target_point_rep.pose.position.z = random_candidate_target.z();
            target_point_rep.pose.orientation.x = 0.0;
            target_point_rep.pose.orientation.y = 0.0;
            target_point_rep.pose.orientation.z = 0.0;
            target_point_rep.pose.orientation.w = 1.0;
            target_point_rep.color.r = 1.0;
            target_point_rep.color.g = 0.0;
            target_point_rep.color.b = 1.0;
            target_point_rep.color.a = 1.0;
            visualization_msgs::MarkerArray points_rep;
            points_rep.markers.push_back(edge_point_rep);
            points_rep.markers.push_back(target_point_rep);
            g_debug_marker_pub.publish(points_rep);
            return true;
        }
        target_checks++;
    }
    ROS_ERROR("No target points found with %u checks", req.query.maximum_target_checks);
    res.result.status = deformable_ompl::ComputeTrialSetupResult::NO_TARGET_POINTS;
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
    DVXLGrid environment("deformable_ompl", DVXL_RESOLUTION, 5.0, 5.0, 5.0, default_dvxl, oob_dvxl);
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

/*
 * Test the new computational digital topology support in the CollisionMapGrid class
 */
void test_CMG_topology()
{
    std::cout << "Testing CollisionMapGrid topology support..." << std::endl;
    std::cout << "Preparing test CMG..." << std::endl;
    sdf_tools::COLLISION_CELL oob_cmg_cell;
    oob_cmg_cell.component = 0;
    oob_cmg_cell.occupancy = 0.0;
    sdf_tools::CollisionMapGrid cmg("test", 1.0, 3.0, 3.0, 3.0, oob_cmg_cell);
    sdf_tools::COLLISION_CELL filled_cmg_cell;
    filled_cmg_cell.component = 0;
    filled_cmg_cell.occupancy = 1.0;
    for (int64_t x_index = 0; x_index < cmg.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < cmg.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < cmg.GetNumZCells(); z_index++)
            {
                if ((x_index == 1 && y_index == 1) || (x_index == 1 && z_index == 1) || (y_index == 1 && z_index == 1))
                //if (x_index == 1 && y_index == 1 && z_index == 1)
                {
                    continue;
                }
                else
                {
                    cmg.Set(x_index, y_index, z_index, filled_cmg_cell);
                }
            }
        }
    }
    std::cout << "Computing connected components and genus..." << std::endl;
    std::map<u_int32_t, std::pair<int32_t, int32_t>> connected_component_topology = cmg.ComputeComponentTopology(true, true, true);
    int32_t num_connected_components = 0;
    int32_t num_holes = 0;
    int32_t num_voids = 0;
    std::map<u_int32_t, std::pair<int32_t, int32_t>>::iterator connected_component_topology_itr;
    for (connected_component_topology_itr = connected_component_topology.begin(); connected_component_topology_itr != connected_component_topology.end(); ++connected_component_topology_itr)
    {
        std::cout << "...component " << connected_component_topology_itr->first << " has " << connected_component_topology_itr->second.first << " holes and " << connected_component_topology_itr->second.second << " voids..." << std::endl;
        num_connected_components++;
        num_holes += connected_component_topology_itr->second.first;
        num_voids += connected_component_topology_itr->second.second;
    }
    std::cout << "...computed " << num_connected_components << " components " << num_holes << " holes and " << num_voids << " voids" << std::endl;
    std::cout << "...done" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deformable_ompl_node");
    ROS_INFO("Starting deformable OMPL planner node...");
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
    ros::ServiceServer planner_server = nh.advertiseService(ros::this_node::getName() + "/plan_path", ProbePlanPathServiceCB);
    ros::ServiceServer dvxl_cost_fn_service = nh.advertiseService(ros::this_node::getName() + "/compute_dvxl_cost", ComputeDVXLCostServiceCB);
    ros::ServiceServer resample_path_service = nh.advertiseService(ros::this_node::getName() + "/resample_path", ResamplePathServiceCB);
    ros::ServiceServer simplify_path_service = nh.advertiseService(ros::this_node::getName() + "/simplify_path", SimplifyPathServiceCB);
    ros::ServiceServer compute_trial_setup_service = nh.advertiseService(ros::this_node::getName() + "/compute_trial_setup", ComputeTrialSetupServiceCB);
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
