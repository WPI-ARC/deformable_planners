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
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include "arc_utilities/voxel_grid.hpp"
#include "deformable_ompl/DvxlGrid.h"

#ifndef SIMPLE_RRT_PLANNER_HPP
#define SIMPLE_RRT_PLANNER_HPP

namespace simple_rrt_planner
{
    template<typename T, typename Allocator=std::allocator<T>>
    class SimpleRRTPlannerState
    {
    protected:

        bool initialized_;
        int64_t parent_index_;
        std::vector<int64_t> child_indices_;
        T value_;

    public:

        SimpleRRTPlannerState() : initialized_(false), parent_index_(-1)
        {
            child_indices_.clear();
        }

        SimpleRRTPlannerState(const T& value, const int64_t parent_index, const std::vector<int64_t>& child_indices)
        {
            parent_index_ = parent_index;
            child_indices_ = child_indices;
            value_ = value;
            initialized_ = true;
        }

        SimpleRRTPlannerState(const T& value, const int64_t parent_index)
        {
            parent_index_ = parent_index;
            child_indices_.clear();
            value_ = value;
            initialized_ = true;
        }

        SimpleRRTPlannerState(const T& value)
        {
            parent_index_ = -1;
            child_indices_.clear();
            value_ = value;
            initialized_ = true;
        }

        const T& GetValueImmutable() const
        {
            return value_;
        }

        T& GetValueMutable()
        {
            return value_;
        }

        int64_t GetParentIndex() const
        {
            return parent_index_;
        }

        void SetParentIndex(const int64_t parent_index)
        {
            parent_index_ = parent_index;
        }

        const std::vector<int64_t>& GetChildIndices()
        {
            return child_indices_;
        }

        void ClearChildIndicies()
        {
            child_indices_.clear();
        }

        void AddChildIndex(const int64_t child_index)
        {
            for (size_t idx = 0; idx < child_indices_.size(); idx++)
            {
                if (child_indices_[idx] == child_index)
                {
                    return;
                }
            }
            child_indices_.push_back(child_index);
        }

        void RemoveChildIndex(const int64_t child_index)
        {
            std::vector<int64_t> new_child_indices;
            for (size_t idx = 0; idx < child_indices_.size(); idx++)
            {
                if (child_indices_[idx] != child_index)
                {
                    new_child_indices.push_back(child_indices_[idx]);
                }
            }
            child_indices_ = new_child_indices;
        }
    };

    template<typename T, typename Allocator=std::allocator<T>>
    class SimpleHybridRRTPlanner
    {
    protected:

        ;

    public:

        std::pair<std::vector<T>, std::map<std::string, double>> Plan(const T& start, const T& goal, std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn, std::function<bool(const T&, const T&)>& goal_reached_fn, std::function<T(void)>& sampling_fn, std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn, std::mt19937_64& rng, const std::chrono::duration<double>& time_limit, const double goal_sampling_probability) const
        {
            // Uniform distribution for goal biasing
            std::uniform_real_distribution<double> goal_biasing_distribution(0.0, 1.0);
            // Keep track of time
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
            std::chrono::time_point<std::chrono::high_resolution_clock> cur_time;
            // Keep track of states
            std::vector<SimpleRRTPlannerState<T, Allocator>> nodes;
            // Add the start state
            SimpleRRTPlannerState<T, Allocator> start_state(start);
            nodes.push_back(start_state);
            // Keep track of statistics
            std::map<std::string, double> statistics;
            statistics["total_samples"] = 0.0;
            statistics["successful_samples"] = 0.0;
            statistics["failed_samples"] = 0.0;
            // Storage for the final planned path
            std::vector<T> planned_path;
            // Update the start time & current time
            start_time = std::chrono::high_resolution_clock::now();
            cur_time = std::chrono::high_resolution_clock::now();
            // Plan
            while (time_limit > (cur_time - start_time))
            {
                // Sample a random goal
                T random_target;
                double sample_goal = goal_biasing_distribution(rng);
                if (sample_goal < goal_sampling_probability)
                {
                    random_target = goal;
                }
                else
                {
                    random_target = sampling_fn();
                }
                // Get the nearest neighbor
                int64_t nearest_neighbor_index = nearest_neighbor_fn(nodes, random_target);
                const T& nearest_neighbor = nodes.at(nearest_neighbor_index).GetValueImmutable();
                // Forward propagate towards the goal
                std::vector<T> propagated = forward_propagation_fn(nearest_neighbor, random_target);
                if (!propagated.empty())
                {
                    statistics["total_samples"] += 1.0;
                    statistics["successful_samples"] += 1.0;
                    int64_t node_parent_index = nearest_neighbor_index;
                    for (size_t idx = 0; idx < propagated.size(); idx++)
                    {
                        const T& current_propagated = propagated[idx];
                        // Check if we've reached the goal
                        if (goal_reached_fn(current_propagated, goal))
                        {
                            planned_path.push_back(current_propagated);
                            int64_t parent_index = node_parent_index;
                            while (parent_index >= 0)
                            {
                                const SimpleRRTPlannerState<T, Allocator>& parent_state = nodes.at(parent_index);
                                const T& parent = parent_state.GetValueImmutable();
                                planned_path.push_back(parent);
                                parent_index = parent_state.GetParentIndex();
                            }
                            std::reverse(planned_path.begin(), planned_path.end());
                            // Update the statistics
                            cur_time = std::chrono::high_resolution_clock::now();
                            statistics["planning_time"] = (cur_time - start_time).count();
                            statistics["total_states"] = nodes.size();
                            statistics["solution_path_length"] = planned_path.size();
                            // Put together the results
                            return std::pair<std::vector<T>, std::map<std::string, double>>(planned_path, statistics);
                        }
                        // If not, add it to the tree
                        else
                        {
                            SimpleRRTPlannerState<T, Allocator> new_state(current_propagated, node_parent_index);
                            nodes.push_back(new_state);
                            node_parent_index = (int64_t)nodes.size() - 1;
                        }
                    }
                }
                else
                {
                    statistics["total_samples"] += 1.0;
                    statistics["failed_samples"] += 1.0;
                }
                // Update the end time
                cur_time = std::chrono::high_resolution_clock::now();
            }
            std::cerr << "Planning time exceeded" << std::endl;
            // Put together the results
            return std::pair<std::vector<T>, std::map<std::string, double>>(planned_path, statistics);
        }
    };

}

#endif // SIMPLE_RRT_PLANNER
