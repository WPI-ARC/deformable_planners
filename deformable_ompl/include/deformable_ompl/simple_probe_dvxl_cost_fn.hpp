#include <unordered_map>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "deformable_ompl/dvxl_cost_fn.hpp"
#include "deformable_ompl/simple_probe_state_space.hpp"
#include "deformable_ompl/SimplePathState.h"
#include "arc_utilities/pretty_print.hpp"

#ifndef SIMPLE_PROBE_DVXL_COST_FN_HPP
#define SIMPLE_PROBE_DVXL_COST_FN_HPP

// Macro to disable unused parameter compiler warnings
#define UNUSED(x) (void)(x)

#define COST_DVXL_RESOLUTION 0.005
#define DVXL_RESOLUTION COST_DVXL_RESOLUTION
#define PUNCTURE_DVXL_RESOLUTION 0.005

#if OMPL_MAJOR_VERSION > 0
    #define GET_OMPL_COST value()
#else
    #define GET_OMPL_COST v
#endif

#define ENV_X_SIZE 0.420
#define ENV_Y_SIZE 0.593

#define ENV_MAX_YAW M_PI
#define ENV_MIN_YAW -(3.0 * M_PI_4)

namespace deformable_ompl
{
    class SimpleProbeDVXLCostValidityChecker : public ompl::base::StateValidityChecker, public ompl::base::MotionValidator, public ompl::base::StateCostIntegralObjective
    {
    protected:

        ompl::base::SpaceInformationPtr space_info_;
        bool verbose_;
        DVXLCostFn dvxl_cost_fn_;
        DVXLCostFn puncture_dvxl_cost_fn_;
        std::vector<std::pair<Eigen::Vector3d, DVXL>> probe_points_;
        std::vector<std::pair<Eigen::Vector3d, DVXL>> puncture_probe_points_;
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> collision_planes_;
        bool disable_intermediate_puncture_check_;

        std::vector<std::pair<Eigen::Vector3d, DVXL>> MakeProbePoints(const double resolution) const;

        DVXLGrid MakePunctureCheckEnvironment(const DVXLGrid&, const double puncture_check_resolution) const;

        std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>> base_object_surfaces_;

        mutable std::unordered_map<std::string, std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>> state_surfaces_;
        mutable int64_t lookups_;

        mutable u_int64_t state_invalid_;
        mutable u_int64_t puncture_invalid_;
        mutable u_int64_t fast_invalid_;

    public:

        SimpleProbeDVXLCostValidityChecker(const ompl::base::SpaceInformationPtr& si, DVXLGrid cost_check_environment, DVXLGrid puncture_check_environment, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& collision_planes) : ompl::base::StateValidityChecker(si), ompl::base::MotionValidator(si), ompl::base::StateCostIntegralObjective(si, true), space_info_(si)
        {
            specs_.clearanceComputationType = ompl::base::StateValidityCheckerSpecs::NONE;
            specs_.hasValidDirectionComputation = false;
            interpolateMotionCost_ = true;
            verbose_ = false;
            dvxl_cost_fn_ = DVXLCostFn(cost_check_environment);
            std::vector<Eigen::Vector3d> local_maxima = dvxl_cost_fn_.ExtractLocalMaxima(2.0);
            std::cout << "Local maxima:\n" << PrettyPrint::PrettyPrint(local_maxima) << std::endl;
            puncture_dvxl_cost_fn_ = DVXLCostFn(puncture_check_environment);
            probe_points_ = MakeProbePoints(COST_DVXL_RESOLUTION);
            puncture_probe_points_ = MakeProbePoints(PUNCTURE_DVXL_RESOLUTION);
            collision_planes_ = collision_planes;
            base_object_surfaces_ = puncture_dvxl_cost_fn_.GetDVXLGrid().ExtractObjectSurfaces();
            lookups_ = 0;
            disable_intermediate_puncture_check_ = true;
            state_invalid_ = 0;
            puncture_invalid_ = 0;
            fast_invalid_ = 0;
        }

        inline std::vector<std::pair<Eigen::Vector3d, DVXL>> GetProbePoints() const
        {
            return probe_points_;
        }

        inline DVXLCostFn& GetInternalCostFn()
        {
            return dvxl_cost_fn_;
        }

        inline DVXLCostFn CopyInternalCostFn() const
        {
            return dvxl_cost_fn_;
        }

        void setVerbosity(bool verbose)
        {
            verbose_ = verbose;
        }

        void setIntermediatePunctureChecks(bool enable)
        {
            disable_intermediate_puncture_check_ = enable;
        }

        std::vector<u_int64_t> GetValidInvalidMovementCounts() const
        {
            return std::vector<u_int64_t>({valid_, invalid_, state_invalid_, puncture_invalid_, fast_invalid_});
        }

        // Invalid states have NAN cost
        virtual bool isValid(const ompl::base::State* state) const
        {
            if (verbose_)
            {
                std::cerr << "Computing state validity" << std::endl;
            }
            // Convert the state to our state type
            const deformable_ompl::SimpleProbeStateSpace::StateType* probe_state = state->as<deformable_ompl::SimpleProbeStateSpace::StateType>();
            // Get the transform
            Eigen::Affine3d probe_transform = probe_state->getEigenTransform();
            // Check it
            return CheckStateTransformValidity(probe_transform);
        }

        bool CheckStateTransformValidity(const Eigen::Affine3d& state_transform) const
        {
            // Make sure we're inside bounds
            for (size_t idx = 0; idx < probe_points_.size(); idx++)
            {
                Eigen::Vector3d point_location = state_transform * probe_points_[idx].first;
                if (point_location.x() < -0.0 || point_location.x() > ENV_X_SIZE)
                {
                    return false;
                }
                if (point_location.y() < -0.0 || point_location.y() > ENV_Y_SIZE)
                {
                    return false;
                }
            }
            // Make sure we're inside the collision planes
            // For every plane, make sure the center is inside the planes
            for (size_t idx = 0; idx < collision_planes_.size(); idx++)
            {
                // Get the plane
                const std::pair<Eigen::Vector3d, Eigen::Vector3d>& current_plane = collision_planes_[idx];
                Eigen::Vector3d plane_point = current_plane.first;
                Eigen::Vector3d plane_normal = current_plane.second;
                // Get the vectors
                Eigen::Vector3d center_vector = state_transform.translation() - plane_point;
                // Get the dot products
                double center_dot = center_vector.dot(plane_normal);
                // Check for collision
                if (center_dot < -0.0)
                {
                    return false;
                }
            }
            // Compute cost
            double dvxl_cost = dvxl_cost_fn_.ComputeIntersectionCost(probe_points_, state_transform);
            if (verbose_)
            {
                std::cerr << "Computed DVXL cost for validity check - " << dvxl_cost << std::endl;
            }
            if (isnan(dvxl_cost))
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        double CheckStateTransformCost(const Eigen::Affine3d& state_transform) const
        {
            // Compute cost
            double dvxl_cost = dvxl_cost_fn_.ComputeIntersectionCost(probe_points_, state_transform);
            if (verbose_)
            {
                std::cerr << "Computed DVXL cost - " << dvxl_cost << std::endl;
            }
            return dvxl_cost;
        }

        // Calculate the DVXL deformation cost of the state
        virtual ompl::base::Cost stateCost(const ompl::base::State* state) const
        {
            assert(interpolateMotionCost_);
            if (verbose_)
            {
                std::cerr << "Computing state DVXL cost" << std::endl;
            }
            // Convert the state to our state type
            const deformable_ompl::SimpleProbeStateSpace::StateType* probe_state = state->as<deformable_ompl::SimpleProbeStateSpace::StateType>();
            // Get the transform
            Eigen::Affine3d probe_transform = probe_state->getEigenTransform();
            // Compute cost
            double dvxl_cost = dvxl_cost_fn_.ComputeIntersectionCost(probe_points_, probe_transform);
            if (verbose_)
            {
                std::cerr << "Computed DVXL cost - " << dvxl_cost << std::endl;
            }
            if (isnan(dvxl_cost))
            {
                return ompl::base::Cost(INFINITY);
            }
            else
            {
                return ompl::base::Cost(dvxl_cost);
            }
        }

        // Calculate the cost of moving between two states - use swept volume to check cost
        virtual ompl::base::Cost motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const
        {
            assert(interpolateMotionCost_);
            if (verbose_)
            {
                std::cerr << "Computing movement DVXL cost" << std::endl;
            }
            // Get a copy of the DVXLCostFn
            DVXLCostFn dvxl_cost_fn_copy = CopyInternalCostFn();
            // Get the number of intermediate states
            unsigned int num_segments = space_info_->getStateSpace()->validSegmentCount(s1, s2);
            if (num_segments == 0)
            {
                return ompl::base::Cost(0.0);
            }
            assert(num_segments >= 1);
            if (verbose_)
            {
                std::cerr << "Movement steps - " << num_segments << std::endl;
            }
            double dvxl_cost = 0.0;
            // First, compute the cost of s1 directly
            const deformable_ompl::SimpleProbeStateSpace::StateType* s1_probe_state = s1->as<deformable_ompl::SimpleProbeStateSpace::StateType>();
            // Get the transform
            Eigen::Affine3d s1_probe_transform = s1_probe_state->getEigenTransform();
            // We're assuming that the planner start state is cost-free, and every other s1 has already had its cost accounted for in
            // computing the cost of the parent motion. Thus, if we included the cost of s1, we'd be double-counting it
//            // Compute cost
//            dvxl_cost += dvxl_cost_fn_copy.ComputeIntersectionCost(probe_points_, s1_probe_transform);
            // Remove the corresponding points to s1 from the DVXL grid
            dvxl_cost_fn_copy.RemovePoints(probe_points_, s1_probe_transform, false);
            // Loop through the path
            // Loop through the intermediate states
            ompl::base::State* intermediate_state = space_info_->allocState();
            for (unsigned int idx = 1; idx <= num_segments; idx++)
            {
                // Interpolate the intermediate state
                space_info_->getStateSpace()->interpolate(s1, s2, (double) idx / (double) num_segments, intermediate_state);
                // Add the cost of the interpolated state
                // First, compute the cost of s1 directly
                const deformable_ompl::SimpleProbeStateSpace::StateType* intermediate_probe_state = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>();
                // Get the transform
                Eigen::Affine3d intermediate_probe_transform = intermediate_probe_state->getEigenTransform();
                // Compute cost
                dvxl_cost += dvxl_cost_fn_copy.ComputeIntersectionCost(probe_points_, intermediate_probe_transform);
                // Remove the corresponding points to s1 from the DVXL grid
                dvxl_cost_fn_copy.RemovePoints(probe_points_, intermediate_probe_transform, false);
            }
            space_info_->freeState(intermediate_state);
            if (verbose_)
            {
                std::cerr << "Computed movement DVXL cost - " << dvxl_cost << std::endl;
            }
            return ompl::base::Cost(dvxl_cost);
        }

        virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
        {
            bool valid = true;
            if (verbose_)
            {
                std::cerr << "Computing movement DVXL validity" << std::endl;
            }
            unsigned int num_segments = space_info_->getStateSpace()->validSegmentCount(s1, s2);
            if (num_segments == 0)
            {
                valid_++;
                valid = true;
                if (verbose_)
                {
                    std::cerr << "Motion check with 0 segments, no puncture" << std::endl;
                }
                return valid;
            }
            assert(num_segments >= 1);
            if (verbose_)
            {
                std::cerr << "Movement steps - " << num_segments << std::endl;
            }
            // Check state s2 for validity before checking the motion - this is cheaper than adding collision plane support via a state sampler
            if (!isValid(s2))
            {
                valid = false;
                if (verbose_)
                {
                    std::cerr << "State s2 is invalid, failed early validity check" << std::endl;
                }
                invalid_++;
                state_invalid_++;
                fast_invalid_++;
                return valid;
            }
            // Get the start state surface cells
            // We know this is puncture free
            std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>> working_surfaces = GetSurfaces(s1);
            const std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>> starting_surfaces = working_surfaces;
            // Loop through the intermediate states
            ompl::base::State* intermediate_state = space_info_->allocState();
            for (unsigned int idx = 1; idx <= num_segments; idx++)
            {
                // Interpolate the intermediate state
                space_info_->getStateSpace()->interpolate(s1, s2, (double) idx / (double) num_segments, intermediate_state);
                if (!isValid(intermediate_state))
                {
                    if (verbose_)
                    {
                        std::cerr << "Stopping motion test, state invalid" << std::endl;
                    }
                    valid = false;
                    invalid_++;
                    state_invalid_++;
                    break;
                }
                // Convert the state to our state type
                const deformable_ompl::SimpleProbeStateSpace::StateType* intermediate_probe_state = intermediate_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>();
                // Get the transform
                Eigen::Affine3d intermediate_probe_transform = intermediate_probe_state->getEigenTransform();
                // Update the surfaces
                puncture_dvxl_cost_fn_.UpdateSurfacesWithIntersectingPoints(puncture_probe_points_, intermediate_probe_transform, working_surfaces, verbose_);
            }
            space_info_->freeState(intermediate_state);
            // Make sure none of the intermediate states were invalid
            if (valid)
            {
                // Now that we've updated surfaces for the entire motion, check for puncture
                bool punctured = puncture_dvxl_cost_fn_.CheckForPuncture(working_surfaces, starting_surfaces, verbose_);
                if (punctured)
                {
                    if (verbose_)
                    {
                        std::cerr << "Stopping motion test, puncture (w/o intermediate checks)" << std::endl;
                    }
                    valid = false;
                    puncture_invalid_++;
                    invalid_++;
                }
                else
                {
                    valid = true;
                    StoreSurfaces(s2, working_surfaces);
                    valid_++;
                }
            }
            if (verbose_)
            {
                if (valid)
                {
                    std::cerr << "Motion validity check - PASS" << std::endl;
                }
                else
                {
                    std::cerr << "Motion validity check - FAIL" << std::endl;
                }
            }
            return valid;
        }

        bool CheckForTopologyChanges(const std::map<u_int32_t, std::pair<int32_t, int32_t>>& original_topology, const std::map<u_int32_t, std::pair<int32_t, int32_t>>& new_topology) const
        {
            // Count the number of components, holes, and voids
            int32_t num_original_components = 0;
            int32_t num_original_holes = 0;
            int32_t num_original_voids = 0;
            std::map<u_int32_t, std::pair<int32_t, int32_t>>::const_iterator original_topology_itr;
            for (original_topology_itr = original_topology.begin(); original_topology_itr != original_topology.end(); ++original_topology_itr)
            {
                num_original_components++;
                num_original_holes += original_topology_itr->second.first;
                num_original_voids += original_topology_itr->second.second;
            }
            int32_t num_new_components = 0;
            int32_t num_new_holes = 0;
            int32_t num_new_voids = 0;
            std::map<u_int32_t, std::pair<int32_t, int32_t>>::const_iterator new_topology_itr;
            for (new_topology_itr = new_topology.begin(); new_topology_itr != new_topology.end(); ++new_topology_itr)
            {
                num_new_components++;
                num_new_holes += new_topology_itr->second.first;
                num_new_voids += new_topology_itr->second.second;
            }
            // Compare
            if (num_original_components == num_new_components && num_original_holes == num_new_holes && num_original_voids == num_new_voids)
            {
                if (true)
                {
                    std::cerr << "Topology match - original topology [" << num_original_components << "," << num_original_holes << "," << num_original_voids << "] with new topology [" << num_new_components << "," << num_new_holes << "," << num_new_voids << "]" << std::endl;
                }
                return true;
            }
            else
            {
                if (true)
                {
                    std::cerr << "Topology mismatch - original topology [" << num_original_components << "," << num_original_holes << "," << num_original_voids << "] with new topology [" << num_new_components << "," << num_new_holes << "," << num_new_voids << "]" << std::endl;
                }
                return false;
            }
        }

        double ComputePathLength(const std::vector<deformable_ompl::SimplePathState>& path) const
        {
            if (path.size() < 2)
            {
                return 0.0;
            }
            double length = 0.0;
            for (size_t idx = 1; idx < path.size(); idx++)
            {
                // Get the raw path states
                const deformable_ompl::SimplePathState& prev_state = path[idx - 1];
                const deformable_ompl::SimplePathState& curr_state = path[idx];
                // Convert them to OMPL states
                ompl::base::State* prev_probe_state = space_info_->allocState();
                prev_probe_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->setPositionAndRotation(prev_state.x, prev_state.y, prev_state.yaw);
                ompl::base::State* curr_probe_state = space_info_->allocState();
                curr_probe_state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->setPositionAndRotation(curr_state.x, curr_state.y, curr_state.yaw);
                // We want to add all the intermediate states to our returned path
                double distance = space_info_->getStateSpace()->distance(prev_probe_state, curr_probe_state);
                length += distance;
                space_info_->freeState(prev_probe_state);
                space_info_->freeState(curr_probe_state);
            }
            return length;
        }

        bool IncrementalCheckFinalPath(const std::vector<deformable_ompl::SimplePathState>& path, ros::Publisher& display_pub) const
        {
            // Get the initial object surfaces
            std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>> object_surfaces = base_object_surfaces_;
            // Loop through the path and update the surfaces
            for (size_t idx = 0; idx < path.size(); idx++)
            {
                Eigen::Translation3d probe_translation(path[idx].x, path[idx].y, 0.0);
                Eigen::Quaterniond probe_orientation(Eigen::AngleAxisd(path[idx].yaw, Eigen::Vector3d::UnitZ()));
                Eigen::Affine3d probe_transform = probe_translation * probe_orientation;
                if (CheckStateTransformValidity(probe_transform))
                {
                    puncture_dvxl_cost_fn_.UpdateSurfacesWithIntersectingPoints(puncture_probe_points_, probe_transform, object_surfaces, verbose_);
                }
                else
                {
                    std::cerr << "Failed final path validity check" << std::endl;
                    return false;
                }
            }
            // Draw the surfaces
            visualization_msgs::Marker surfaces_display_rep = puncture_dvxl_cost_fn_.GetDVXLGrid().ExportSurfacesForDisplay(object_surfaces);
            surfaces_display_rep.ns = "final_puncture_check";
            visualization_msgs::MarkerArray display_reps;
            display_reps.markers.push_back(surfaces_display_rep);
            display_pub.publish(display_reps);
            // Now that the collision map has been processed, check for puncture
            bool punctured = puncture_dvxl_cost_fn_.CheckForPuncture(object_surfaces, true);
            if (punctured)
            {
                std::cerr << "Failed final path puncture check" << std::endl;
                return false;
            }
            else
            {
                std::cerr << "Passed final path puncture check" << std::endl;
                return true;
            }
        }

        double ComputeCostFinalPath(const std::vector<deformable_ompl::SimplePathState>& path, ros::Publisher& display_pub) const
        {
            // Get a copy of the DVXLCostFn
            DVXLCostFn dvxl_cost_fn_copy = CopyInternalCostFn();
            double path_dvxl_cost = 0.0;
            for (size_t idx = 0; idx < path.size(); idx++)
            {
                Eigen::Translation3d path_state_translation(path[idx].x, path[idx].y, 0.0);
                Eigen::Quaterniond path_state_rotation(Eigen::AngleAxisd(path[idx].yaw, Eigen::Vector3d::UnitZ()));
                Eigen::Affine3d path_state_transform = path_state_translation * path_state_rotation;
                double path_state_cost = dvxl_cost_fn_copy.ComputeIntersectionCost(probe_points_, path_state_transform);
                dvxl_cost_fn_copy.RemovePoints(probe_points_, path_state_transform, false);
                path_dvxl_cost += path_state_cost;
            }
            // Draw the objects with swept volume removed
            visualization_msgs::Marker env_display_rep = dvxl_cost_fn_copy.GetDVXLGrid().ExportForDisplay();
            env_display_rep.header.frame_id = "deformable_ompl";
            env_display_rep.ns = "final_cost_check";
            visualization_msgs::MarkerArray display_reps;
            display_reps.markers.push_back(env_display_rep);
            display_pub.publish(display_reps);
            std::cerr << "Computed final path DVXL cost " << path_dvxl_cost << std::endl;
            return path_dvxl_cost;
        }

        std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>> GetSurfaces(const ompl::base::State* state) const
        {
            std::unordered_map<std::string, std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>>::const_iterator found_surfaces = state_surfaces_.find(state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getKey());
            // If not found, return a copy of the base object surfaces
            if (found_surfaces == state_surfaces_.end())
            {
                return base_object_surfaces_;
            }
            else
            {
                return found_surfaces->second;
            }
        }

        void StoreSurfaces(const ompl::base::State* state, std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces) const
        {
            state_surfaces_[state->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getKey()] = surfaces;
        }

        virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State*, double> &lastValid) const
        {
            UNUSED(s1);
            UNUSED(s2);
            UNUSED(lastValid);
            std::cerr << "DO NOT USE THIS MOTION CHECK FUNCTION" << std::endl;
            throw std::invalid_argument("checkMotion(s1, s2, lastValid) is not safely implemented");
        }
    };
}

#endif // SIMPLE_PROBE_DVXL_COST_FN_HPP
