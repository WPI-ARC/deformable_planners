#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <random>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include "arc_utilities/eigen_helpers.hpp"
#include "arc_utilities/pretty_print.hpp"
#include "arc_utilities/voxel_grid.hpp"
#include "deformable_ompl/DvxlGrid.h"
#include "deformable_ompl/simple_rrt_planner.hpp"
#include "deformable_ompl/simple_pid_controller.hpp"
#include "deformable_ompl/simple_uncertainty_models.hpp"

//typedef EigenHelpers::VectorVector3d nomdp_planner_state;
typedef Eigen::Vector3d nomdp_planner_state;
typedef std::function<int64_t(const std::vector<simple_rrt_planner::SimpleRRTPlannerState<nomdp_planner_state>>&, const nomdp_planner_state&)> nomdp_nearest_neighbor_fn;
typedef std::function<bool(const nomdp_planner_state&)> nomdp_goal_reached_fn;
typedef std::function<nomdp_planner_state(void)> nomdp_sampling_fn;
typedef std::function<std::vector<nomdp_planner_state>(const nomdp_planner_state&, const nomdp_planner_state&)> nomdp_propagation_fn;

class SimpleRRTHelpers
{
protected:

    double step_size_;
    std::mt19937_64 rng_;
    double min_x_;
    double min_y_;
    double min_z_;
    double max_x_;
    double max_y_;
    double max_z_;

public:

    SimpleRRTHelpers(const Eigen::Vector3d& min_bounds, const Eigen::Vector3d& max_bounds, const double step_size, std::mt19937_64& rng) : rng_(rng)
    {
        step_size_ = step_size;
        min_x_ = min_bounds.x();
        min_y_ = min_bounds.y();
        min_z_ = min_bounds.z();
        max_x_ = max_bounds.x();
        max_y_ = max_bounds.y();
        max_z_ = max_bounds.z();
    }

    int64_t GetNearestNeighbor(const std::vector<simple_rrt_planner::SimpleRRTPlannerState<nomdp_planner_state>>& planner_nodes, const nomdp_planner_state& random_state)
    {
        double min_distance = INFINITY;
        int64_t min_index = -1;
        for (size_t idx = 0; idx < planner_nodes.size(); idx++)
        {
            const simple_rrt_planner::SimpleRRTPlannerState<nomdp_planner_state>& current_node = planner_nodes[idx];
            const nomdp_planner_state& current_state = current_node.GetValueImmutable();
            double distance = (current_state - random_state).norm();
            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = idx;
            }
        }
        return min_index;
    }

    nomdp_planner_state SampleRandomTarget()
    {
        std::uniform_real_distribution<double> x_distribution(min_x_, max_x_);
        std::uniform_real_distribution<double> y_distribution(min_y_, max_y_);
        std::uniform_real_distribution<double> z_distribution(min_z_, max_z_);
        double rand_x = x_distribution(rng_);
        double rand_y = y_distribution(rng_);
        double rand_z = z_distribution(rng_);
        nomdp_planner_state rand_target(rand_x, rand_y, rand_z);
        return rand_target;
    }

    std::vector<nomdp_planner_state> ExtendPropagateForwards(const nomdp_planner_state& current_state, const nomdp_planner_state& target_state)
    {
        Eigen::Vector3d raw_extension_vector = target_state - current_state;
        Eigen::Vector3d extension_unit_vector = raw_extension_vector / raw_extension_vector.norm();
        Eigen::Vector3d extension_vector = extension_unit_vector * step_size_;
        nomdp_planner_state propagated_state = current_state + extension_vector;
        if (IsValid(propagated_state))
        {
            return std::vector<nomdp_planner_state>{propagated_state};
        }
        else
        {
            return std::vector<nomdp_planner_state>();
        }
    }

    std::vector<nomdp_planner_state> ConnectPropagateForwards(const nomdp_planner_state& start_state, const nomdp_planner_state& target_state)
    {
        std::vector<nomdp_planner_state> connect_states;
        Eigen::Vector3d raw_connect_vector = target_state - start_state;
        Eigen::Vector3d extension_unit_vector = raw_connect_vector / raw_connect_vector.norm();
        Eigen::Vector3d extension_vector = extension_unit_vector * step_size_;
        double connect_distance = raw_connect_vector.norm();
        nomdp_planner_state current_state = start_state;
        while (connect_distance > step_size_)
        {
            nomdp_planner_state propagated_state = current_state + extension_vector;
            if (IsValid(propagated_state))
            {
                connect_states.push_back(propagated_state);
                current_state = propagated_state;
                connect_distance = (target_state - current_state).norm();
            }
            else
            {
                return connect_states;
            }
        }
        if (IsValid(target_state))
        {
            connect_states.push_back(target_state);
        }
        return connect_states;
    }

    bool IsValid(const nomdp_planner_state& state)
    {
        if (state.x() > 3.0 && state.x() < 7.0)
        {
            if (state.y() > 3.0 && state.y() < 7.0)
            {
                if (state.z() > 3.0 && state.z() < 7.0)
                {
                    return false;
                }
            }
        }
        return true;
    }
};



int main(int argc, char** argv)
{
    for (int idx = 0; idx < argc; idx++)
    {
        printf("Argument %d: %s\n", idx + 1, argv[idx]);
    }
    // Stuff
    unsigned long seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937_64 prng(seed);
    std::uniform_real_distribution<double> goal_sampling_distribution(0.0, 1.0);
    simple_rrt_planner::SimpleHybridRRTPlanner<nomdp_planner_state> planner;
    // Make the functions
    double goal_bias = 0.05;
    double goal_tolerance = 0.05;
    double step_size = 0.05;
    nomdp_planner_state start_state(0.1, 0.1, 0.1);
    nomdp_planner_state goal_state(9.9, 9.9, 9.9);
    std::chrono::duration<double> time_limit(10.0);
    Eigen::Vector3d min_bounds(0.0, 0.0, 0.0);
    Eigen::Vector3d max_bounds(10.0, 10.0, 10.0);
    SimpleRRTHelpers planner_fn_class(min_bounds, max_bounds, step_size, prng);
    nomdp_nearest_neighbor_fn nearest_neighbor_fn = std::bind(&SimpleRRTHelpers::GetNearestNeighbor, &planner_fn_class, std::placeholders::_1, std::placeholders::_2);
    nomdp_goal_reached_fn goal_reached_fn = [&](const nomdp_planner_state& current_state) { return ((current_state - goal_state).norm() < goal_tolerance); };



    nomdp_sampling_fn state_sampling_fn = std::bind(&SimpleRRTHelpers::SampleRandomTarget, &planner_fn_class);
    nomdp_sampling_fn sampling_fn = [&]() { return ((goal_sampling_distribution(prng) < goal_bias) ? state_sampling_fn() : goal_state); };


    nomdp_propagation_fn forward_propagation_fn = std::bind(&SimpleRRTHelpers::ConnectPropagateForwards, &planner_fn_class, std::placeholders::_1, std::placeholders::_2);
    // Plan
    std::pair<std::vector<nomdp_planner_state>, std::map<std::string, double>> planner_result = planner.Plan(start_state, nearest_neighbor_fn, goal_reached_fn, sampling_fn, forward_propagation_fn, time_limit);
    std::cout << "Planner results: " << PrettyPrint::PrettyPrint(planner_result.second) << std::endl;
    std::cout << "Planned path:\n" << PrettyPrint::PrettyPrint(planner_result.first, false, "\n") << std::endl;
    return 0;
}
