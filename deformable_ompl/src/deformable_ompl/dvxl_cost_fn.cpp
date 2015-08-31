#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include "deformable_ompl/dvxl_grid.hpp"
#include "deformable_ompl/dvxl_cost_fn.hpp"

using namespace deformable_ompl;

DVXLCostFn::DVXLCostFn(std::string environment_filepath)
{
    bool loaded = environment_.LoadFromFile(environment_filepath);
    if (loaded)
    {
        std::vector<u_int32_t> all_objects;
        std::pair<sdf_tools::SignedDistanceField, sdf_tools::CollisionMapGrid> grids = UpdateInternalGrids(environment_, all_objects);
        sdf_ = grids.first;
        collision_map_ = grids.second;
        // Make an SDF for each object
        std::vector<u_int32_t> object_ids = GetAllObjectIDs(environment_);
        for (size_t idx = 0; idx < object_ids.size(); idx++)
        {
            std::vector<u_int32_t> current_object{object_ids[idx]};
            sdf_tools::SignedDistanceField object_sdf = UpdateInternalGrids(environment_, current_object).first;
            object_sdfs_[object_ids[idx]] = object_sdf;
        }
        initialized_ = true;
    }
    else
    {
        throw std::invalid_argument("Invalid environment file provided, could not load");
    }
}

DVXLCostFn::DVXLCostFn(DVXLGrid environment)
{
    environment_ = environment;
    std::vector<u_int32_t> all_objects;
    std::pair<sdf_tools::SignedDistanceField, sdf_tools::CollisionMapGrid> grids = UpdateInternalGrids(environment_, all_objects);
    sdf_ = grids.first;
    collision_map_ = grids.second;
    // Make an SDF for each object
    std::vector<u_int32_t> object_ids = GetAllObjectIDs(environment_);
    for (size_t idx = 0; idx < object_ids.size(); idx++)
    {
        std::vector<u_int32_t> current_object{object_ids[idx]};
        sdf_tools::SignedDistanceField object_sdf = UpdateInternalGrids(environment_, current_object).first;
        object_sdfs_[object_ids[idx]] = object_sdf;
    }
    initialized_ = true;
}

std::vector<u_int32_t> DVXLCostFn::GetAllObjectIDs(const DVXLGrid& environment)
{
    std::map<u_int32_t, u_int8_t> object_ids;
    for (int64_t x_idx = 0; x_idx < environment.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < environment.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < environment.GetNumZCells(); z_idx++)
            {
                u_int32_t object_id = environment.GetImmutable(x_idx, y_idx, z_idx).first.object_id;
                if (object_id > 0)
                {
                    object_ids[object_id] = 1;
                }
            }
        }
    }
    std::vector<u_int32_t> all_object_ids;
    std::map<u_int32_t, u_int8_t>::const_iterator object_ids_itr;
    for (object_ids_itr = object_ids.begin(); object_ids_itr != object_ids.end(); ++object_ids_itr)
    {
        u_int32_t object_id = object_ids_itr->first;
        u_int8_t val = object_ids_itr->second;
        if (val > 0)
        {
            all_object_ids.push_back(object_id);
        }
    }
    return all_object_ids;
}

std::vector<Eigen::Vector3d> DVXLCostFn::ExtractMinimumVoronoiPoints(const u_int32_t first_object_id, const u_int32_t second_object_id) const
{
    // First, make sure both objects are present
    std::map<u_int32_t, sdf_tools::SignedDistanceField>::const_iterator found_first_object_sdf = object_sdfs_.find(first_object_id);
    if (found_first_object_sdf == object_sdfs_.end())
    {
        std::cerr << "First object is not in the environment" << std::endl;
        return std::vector<Eigen::Vector3d>();
    }
    std::map<u_int32_t, sdf_tools::SignedDistanceField>::const_iterator found_second_object_sdf = object_sdfs_.find(second_object_id);
    if (found_second_object_sdf == object_sdfs_.end())
    {
        std::cerr << "Second object is not in the environment" << std::endl;
        return std::vector<Eigen::Vector3d>();
    }
    // Get the SDFs
    const sdf_tools::SignedDistanceField& first_object_sdf = found_first_object_sdf->second;
    const sdf_tools::SignedDistanceField& second_object_sdf = found_second_object_sdf->second;
    // Finds the "minimum Voronoi points" between two objects, i.e. the minimum-distance point(s) eqidistant between the objects
    std::vector<Eigen::Vector3d> candidate_minimum_voronoi_points;
    double minimum_distance = INFINITY;
    // Loop through the grid
    for (int64_t x_idx = 0; x_idx < environment_.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < environment_.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < environment_.GetNumZCells(); z_idx++)
            {
                float first_object_distance = first_object_sdf.Get(x_idx, y_idx, z_idx);
                float second_object_distance = second_object_sdf.Get(x_idx, y_idx, z_idx);
                // Make sure we're outside the objects
                if (first_object_distance >= 0.0 && second_object_distance >= 0.0)
                {
                    // Check if we're on a Voronoi boundary (the distances are close enough [accounting for discretization effects])
                    if (fabs(first_object_distance - second_object_distance) <= (environment_.GetResolution() + 0.000001))
                    {
                        // Check if the distance is better that what we've seen so far
                        double effective_distance = std::min(first_object_distance, second_object_distance);
                        Eigen::Vector3d real_location = MakeEigenVector3d(environment_.GridIndexToLocation(x_idx, y_idx, z_idx));
                        // If it's close enough, add it as another point
                        if (CLOSE(effective_distance, minimum_distance))
                        {
                            candidate_minimum_voronoi_points.push_back(real_location);
                        }
                        // If it's closer, replace what we've found already
                        else if (effective_distance < minimum_distance)
                        {
                            minimum_distance = effective_distance;
                            candidate_minimum_voronoi_points.clear();
                            candidate_minimum_voronoi_points.push_back(real_location);
                        }
                    }
                }
            }
        }
    }
    return candidate_minimum_voronoi_points;
}

void DVXLCostFn::RemovePoints(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const bool remove_neighbors)
{
    DVXL empty_dvxl;
    empty_dvxl.sensitivity = 0.0;
    empty_dvxl.deformability = 1.0;
    empty_dvxl.mass = 0.0;
    empty_dvxl.r = 0x00;
    empty_dvxl.g = 0x00;
    empty_dvxl.b = 0x00;
    empty_dvxl.a = 0x00;
    empty_dvxl.object_id = 0;
    for (size_t idx = 0; idx < points.size(); idx++)
    {
        const Eigen::Vector3d& point_location = points[idx].first;
        // Remove from the grid, along with all 26 neighbors
        std::vector<int64_t> location_index = environment_.LocationToGridIndex(point_location.x(), point_location.y(), point_location.z());
        if (location_index.size() == 3)
        {
            int64_t x_index = location_index[0];
            int64_t y_index = location_index[1];
            int64_t z_index = location_index[2];
            environment_.Set(x_index, y_index, z_index, empty_dvxl);
            if (remove_neighbors)
            {
                environment_.Set(x_index - 1, y_index - 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index - 1, z_index, empty_dvxl);
                environment_.Set(x_index - 1, y_index - 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index, z_index - 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index, z_index, empty_dvxl);
                environment_.Set(x_index - 1, y_index, z_index + 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index + 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index + 1, z_index, empty_dvxl);
                environment_.Set(x_index - 1, y_index + 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index, y_index - 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index, y_index - 1, z_index, empty_dvxl);
                environment_.Set(x_index, y_index - 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index, y_index, z_index - 1, empty_dvxl);
                environment_.Set(x_index, y_index, z_index + 1, empty_dvxl);
                environment_.Set(x_index, y_index + 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index, y_index + 1, z_index, empty_dvxl);
                environment_.Set(x_index, y_index + 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index - 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index - 1, z_index, empty_dvxl);
                environment_.Set(x_index + 1, y_index - 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index, z_index - 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index, z_index, empty_dvxl);
                environment_.Set(x_index + 1, y_index, z_index + 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index + 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index + 1, z_index, empty_dvxl);
                environment_.Set(x_index + 1, y_index + 1, z_index + 1, empty_dvxl);
            }
        }
    }
}

void DVXLCostFn::RemovePoints(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const Eigen::Affine3d base_transform, const bool remove_neighbors)
{
    DVXL empty_dvxl;
    empty_dvxl.sensitivity = 0.0;
    empty_dvxl.deformability = 1.0;
    empty_dvxl.mass = 0.0;
    empty_dvxl.r = 0x00;
    empty_dvxl.g = 0x00;
    empty_dvxl.b = 0x00;
    empty_dvxl.a = 0x00;
    empty_dvxl.object_id = 0;
    for (size_t idx = 0; idx < points.size(); idx++)
    {
        Eigen::Vector3d point_location = base_transform * points[idx].first;
        // Remove from the grid, along with all 26 neighbors
        std::vector<int64_t> location_index = environment_.LocationToGridIndex(point_location.x(), point_location.y(), point_location.z());
        if (location_index.size() == 3)
        {
            int64_t x_index = location_index[0];
            int64_t y_index = location_index[1];
            int64_t z_index = location_index[2];
            environment_.Set(x_index, y_index, z_index, empty_dvxl);
            if (remove_neighbors)
            {
                environment_.Set(x_index - 1, y_index - 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index - 1, z_index, empty_dvxl);
                environment_.Set(x_index - 1, y_index - 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index, z_index - 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index, z_index, empty_dvxl);
                environment_.Set(x_index - 1, y_index, z_index + 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index + 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index - 1, y_index + 1, z_index, empty_dvxl);
                environment_.Set(x_index - 1, y_index + 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index, y_index - 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index, y_index - 1, z_index, empty_dvxl);
                environment_.Set(x_index, y_index - 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index, y_index, z_index - 1, empty_dvxl);
                environment_.Set(x_index, y_index, z_index + 1, empty_dvxl);
                environment_.Set(x_index, y_index + 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index, y_index + 1, z_index, empty_dvxl);
                environment_.Set(x_index, y_index + 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index - 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index - 1, z_index, empty_dvxl);
                environment_.Set(x_index + 1, y_index - 1, z_index + 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index, z_index - 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index, z_index, empty_dvxl);
                environment_.Set(x_index + 1, y_index, z_index + 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index + 1, z_index - 1, empty_dvxl);
                environment_.Set(x_index + 1, y_index + 1, z_index, empty_dvxl);
                environment_.Set(x_index + 1, y_index + 1, z_index + 1, empty_dvxl);
            }
        }
    }
}

void DVXLCostFn::RemovePoints(const DVXLGrid& other_grid, const bool remove_neighbors)
{
    DVXL empty_dvxl;
    empty_dvxl.sensitivity = 0.0;
    empty_dvxl.deformability = 1.0;
    empty_dvxl.mass = 0.0;
    empty_dvxl.r = 0x00;
    empty_dvxl.g = 0x00;
    empty_dvxl.b = 0x00;
    empty_dvxl.a = 0x00;
    empty_dvxl.object_id = 0;
    for (int64_t x_index = 0; x_index < other_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < other_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < other_grid.GetNumZCells(); z_index++)
            {
                std::vector<double> raw_relative_other_location = other_grid.GridIndexToLocation(x_index, y_index, z_index);
                // Remove from the grid, along with all 26 neighbors
                std::vector<int64_t> location_index = environment_.LocationToGridIndex(raw_relative_other_location[0], raw_relative_other_location[1], raw_relative_other_location[2]);
                if (location_index.size() == 3)
                {
                    int64_t x_index = location_index[0];
                    int64_t y_index = location_index[1];
                    int64_t z_index = location_index[2];
                    environment_.Set(x_index, y_index, z_index, empty_dvxl);
                    if (remove_neighbors)
                    {
                        environment_.Set(x_index - 1, y_index - 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index - 1, z_index, empty_dvxl);
                        environment_.Set(x_index - 1, y_index - 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index, z_index - 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index, z_index, empty_dvxl);
                        environment_.Set(x_index - 1, y_index, z_index + 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index + 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index + 1, z_index, empty_dvxl);
                        environment_.Set(x_index - 1, y_index + 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index, y_index - 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index, y_index - 1, z_index, empty_dvxl);
                        environment_.Set(x_index, y_index - 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index, y_index, z_index - 1, empty_dvxl);
                        environment_.Set(x_index, y_index, z_index + 1, empty_dvxl);
                        environment_.Set(x_index, y_index + 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index, y_index + 1, z_index, empty_dvxl);
                        environment_.Set(x_index, y_index + 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index - 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index - 1, z_index, empty_dvxl);
                        environment_.Set(x_index + 1, y_index - 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index, z_index - 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index, z_index, empty_dvxl);
                        environment_.Set(x_index + 1, y_index, z_index + 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index + 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index + 1, z_index, empty_dvxl);
                        environment_.Set(x_index + 1, y_index + 1, z_index + 1, empty_dvxl);
                    }
                }
            }
        }
    }
}

void DVXLCostFn::RemovePoints(const DVXLGrid& other_grid, const Eigen::Affine3d base_transform, const bool remove_neighbors)
{
    DVXL empty_dvxl;
    empty_dvxl.sensitivity = 0.0;
    empty_dvxl.deformability = 1.0;
    empty_dvxl.mass = 0.0;
    empty_dvxl.r = 0x00;
    empty_dvxl.g = 0x00;
    empty_dvxl.b = 0x00;
    empty_dvxl.a = 0x00;
    empty_dvxl.object_id = 0;
    for (int64_t x_index = 0; x_index < other_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < other_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < other_grid.GetNumZCells(); z_index++)
            {
                std::vector<double> raw_relative_other_location = other_grid.GridIndexToLocation(x_index, y_index, z_index);
                Eigen::Vector3d relative_other_location(raw_relative_other_location[0], raw_relative_other_location[1], raw_relative_other_location[2]);
                Eigen::Vector3d other_location = base_transform * relative_other_location;
                // Remove from the grid, along with all 26 neighbors
                std::vector<int64_t> location_index = environment_.LocationToGridIndex(other_location.x(), other_location.y(), other_location.z());
                if (location_index.size() == 3)
                {
                    int64_t x_index = location_index[0];
                    int64_t y_index = location_index[1];
                    int64_t z_index = location_index[2];
                    environment_.Set(x_index, y_index, z_index, empty_dvxl);
                    if (remove_neighbors)
                    {
                        environment_.Set(x_index - 1, y_index - 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index - 1, z_index, empty_dvxl);
                        environment_.Set(x_index - 1, y_index - 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index, z_index - 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index, z_index, empty_dvxl);
                        environment_.Set(x_index - 1, y_index, z_index + 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index + 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index - 1, y_index + 1, z_index, empty_dvxl);
                        environment_.Set(x_index - 1, y_index + 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index, y_index - 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index, y_index - 1, z_index, empty_dvxl);
                        environment_.Set(x_index, y_index - 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index, y_index, z_index - 1, empty_dvxl);
                        environment_.Set(x_index, y_index, z_index + 1, empty_dvxl);
                        environment_.Set(x_index, y_index + 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index, y_index + 1, z_index, empty_dvxl);
                        environment_.Set(x_index, y_index + 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index - 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index - 1, z_index, empty_dvxl);
                        environment_.Set(x_index + 1, y_index - 1, z_index + 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index, z_index - 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index, z_index, empty_dvxl);
                        environment_.Set(x_index + 1, y_index, z_index + 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index + 1, z_index - 1, empty_dvxl);
                        environment_.Set(x_index + 1, y_index + 1, z_index, empty_dvxl);
                        environment_.Set(x_index + 1, y_index + 1, z_index + 1, empty_dvxl);
                    }
                }
            }
        }
    }
}

double DVXLCostFn::ComputeIntersectionCost(const DVXLGrid& other_grid) const
{
    double total_intersection_cost = 0.0;
    for (int64_t x_index = 0; x_index < other_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < other_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < other_grid.GetNumZCells(); z_index++)
            {
                DVXL other_dvxl = other_grid.GetImmutable(x_index, y_index, z_index).first;
                std::vector<double> raw_other_location = other_grid.GridIndexToLocation(x_index, y_index, z_index);
                // Lookup in the grid
                DVXL stored_dvxl = environment_.GetImmutable(raw_other_location[0], raw_other_location[1], raw_other_location[2]).first;
                // Compute the intersection cost
                double intersection_cost = DVXLCost(stored_dvxl, other_dvxl);
                total_intersection_cost += intersection_cost;
            }
        }
    }
    return total_intersection_cost;
}

double DVXLCostFn::ComputeIntersectionCost(const DVXLGrid& other_grid, const Eigen::Affine3d base_transform) const
{
    double total_intersection_cost = 0.0;
    for (int64_t x_index = 0; x_index < other_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < other_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < other_grid.GetNumZCells(); z_index++)
            {
                DVXL other_dvxl = other_grid.GetImmutable(x_index, y_index, z_index).first;
                std::vector<double> raw_relative_other_location = other_grid.GridIndexToLocation(x_index, y_index, z_index);
                Eigen::Vector3d relative_other_location(raw_relative_other_location[0], raw_relative_other_location[1], raw_relative_other_location[2]);
                Eigen::Vector3d other_location = base_transform * relative_other_location;
                // Lookup in the grid
                DVXL stored_dvxl = environment_.GetImmutable(other_location.x(), other_location.y(), other_location.z()).first;
                // Compute the intersection cost
                double intersection_cost = DVXLCost(stored_dvxl, other_dvxl);
                total_intersection_cost += intersection_cost;
            }
        }
    }
    return total_intersection_cost;
}

double DVXLCostFn::ComputeIntersectionCost(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points) const
{
    double total_intersection_cost = 0.0;
    for (size_t idx = 0; idx < points.size(); idx++)
    {
        const Eigen::Vector3d& point_location = points[idx].first;
        const DVXL& point_dvxl = points[idx].second;
        // Lookup in the grid
        DVXL stored_dvxl = environment_.GetImmutable(point_location.x(), point_location.y(), point_location.z()).first;
        // Compute the intersection cost
        double intersection_cost = DVXLCost(stored_dvxl, point_dvxl);
        total_intersection_cost += intersection_cost;
    }
    return total_intersection_cost;
}

double DVXLCostFn::ComputeIntersectionCost(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const Eigen::Affine3d base_transform) const
{
    double total_intersection_cost = 0.0;
    for (size_t idx = 0; idx < points.size(); idx++)
    {
        Eigen::Vector3d point_location = base_transform * points[idx].first;
        const DVXL& point_dvxl = points[idx].second;
        // Lookup in the grid
        DVXL stored_dvxl = environment_.GetImmutable(point_location.x(), point_location.y(), point_location.z()).first;
        // Compute the intersection cost
        double intersection_cost = DVXLCost(stored_dvxl, point_dvxl);
        total_intersection_cost += intersection_cost;
    }
    return total_intersection_cost;
}

std::map<u_int32_t, std::vector<COLLISION_INFO>> DVXLCostFn::GetCollisionInfo(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points) const
{
    // Get the collision info for each point
    std::map<u_int32_t, std::vector<COLLISION_INFO>> collision_information;
    for (size_t idx = 0; idx < points.size(); idx++)
    {
        const Eigen::Vector3d& point_location = points[idx].first;
        // Lookup in the grid
        DVXL stored_dvxl = environment_.GetImmutable(point_location.x(), point_location.y(), point_location.z()).first;
        // We only care about colliding points
        if (stored_dvxl.sensitivity > 0.0)
        {
            COLLISION_INFO info;
            info.position = point_location;
            info.component = collision_map_.Get(point_location.x(), point_location.y(), point_location.z()).first.component;
            info.penetration = sdf_.GetSafe(point_location.x(), point_location.y(), point_location.z()).first;
            std::vector<double> raw_gradient = sdf_.GetGradient(point_location.x(), point_location.y(), point_location.z(), true);
            Eigen::Vector3d gradient(raw_gradient[0], raw_gradient[1], raw_gradient[2]);
            info.gradient = gradient;
            collision_information[info.component].push_back(info);
        }
    }
    // Return
    return collision_information;
}

std::map<u_int32_t, std::vector<COLLISION_INFO>> DVXLCostFn::GetCollisionInfo(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const Eigen::Affine3d base_transform) const
{
    // Get the collision info for each point
    std::map<u_int32_t, std::vector<COLLISION_INFO>> collision_information;
    for (size_t idx = 0; idx < points.size(); idx++)
    {
        Eigen::Vector3d point_location = base_transform * points[idx].first;
        // Lookup in the grid
        DVXL stored_dvxl = environment_.GetImmutable(point_location.x(), point_location.y(), point_location.z()).first;
        // We only care about colliding points
        if (stored_dvxl.sensitivity > 0.0)
        {
            COLLISION_INFO info;
            info.position = point_location;
            info.component = collision_map_.Get(point_location.x(), point_location.y(), point_location.z()).first.component;
            info.penetration = sdf_.GetSafe(point_location.x(), point_location.y(), point_location.z()).first;
            std::vector<double> raw_gradient = sdf_.GetGradient(point_location.x(), point_location.y(), point_location.z(), true);
            Eigen::Vector3d gradient(raw_gradient[0], raw_gradient[1], raw_gradient[2]);
            info.gradient = gradient;
            collision_information[info.component].push_back(info);
        }
    }
    // Return
    return collision_information;
}

std::map<u_int32_t, std::vector<COLLISION_INFO>> DVXLCostFn::GetCollisionInfo(const DVXLGrid& other_grid) const
{
    // Get the collision info for each point
    std::map<u_int32_t, std::vector<COLLISION_INFO>> collision_information;
    for (int64_t x_index = 0; x_index < other_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < other_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < other_grid.GetNumZCells(); z_index++)
            {
                std::vector<double> raw_other_location = other_grid.GridIndexToLocation(x_index, y_index, z_index);
                DVXL stored_dvxl = environment_.GetImmutable(raw_other_location[0], raw_other_location[1], raw_other_location[2]).first;
                // We only care about colliding points
                if (stored_dvxl.sensitivity > 0.0)
                {
                    COLLISION_INFO info;
                    info.position = Eigen::Vector3d(raw_other_location[0], raw_other_location[1], raw_other_location[2]);
                    info.component = collision_map_.Get(raw_other_location[0], raw_other_location[1], raw_other_location[2]).first.component;
                    info.penetration = sdf_.GetSafe(raw_other_location[0], raw_other_location[1], raw_other_location[2]).first;
                    std::vector<double> raw_gradient = sdf_.GetGradient(raw_other_location[0], raw_other_location[1], raw_other_location[2], true);
                    Eigen::Vector3d gradient(raw_gradient[0], raw_gradient[1], raw_gradient[2]);
                    info.gradient = gradient;
                    collision_information[info.component].push_back(info);
                }
            }
        }
    }
    // Return
    return collision_information;
}

std::map<u_int32_t, std::vector<COLLISION_INFO>> DVXLCostFn::GetCollisionInfo(const DVXLGrid& other_grid, const Eigen::Affine3d base_transform) const
{
    // Get the collision info for each point
    std::map<u_int32_t, std::vector<COLLISION_INFO>> collision_information;
    for (int64_t x_index = 0; x_index < other_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < other_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < other_grid.GetNumZCells(); z_index++)
            {
                std::vector<double> raw_relative_other_location = other_grid.GridIndexToLocation(x_index, y_index, z_index);
                Eigen::Vector3d relative_other_location(raw_relative_other_location[0], raw_relative_other_location[1], raw_relative_other_location[2]);
                Eigen::Vector3d other_location = base_transform * relative_other_location;
                // Lookup in the grid
                DVXL stored_dvxl = environment_.GetImmutable(other_location.x(), other_location.y(), other_location.z()).first;
                // We only care about colliding points
                if (stored_dvxl.sensitivity > 0.0)
                {
                    COLLISION_INFO info;
                    info.position = other_location;
                    info.component = collision_map_.Get(other_location.x(), other_location.y(), other_location.z()).first.component;
                    info.penetration = sdf_.GetSafe(other_location.x(), other_location.y(), other_location.z()).first;
                    std::vector<double> raw_gradient = sdf_.GetGradient(other_location.x(), other_location.y(), other_location.z(), true);
                    Eigen::Vector3d gradient(raw_gradient[0], raw_gradient[1], raw_gradient[2]);
                    info.gradient = gradient;
                    collision_information[info.component].push_back(info);
                }
            }
        }
    }
    // Return
    return collision_information;
}

std::pair<sdf_tools::SignedDistanceField, sdf_tools::CollisionMapGrid> DVXLCostFn::UpdateInternalGrids(DVXLGrid& environment, std::vector<u_int32_t>& objects_to_use)
{
    // Put the objects in a map for quick checking
    std::map<u_int32_t, u_int8_t> object_map;
    for (size_t idx = 0; idx < objects_to_use.size(); idx++)
    {
        object_map[objects_to_use[idx]] = 1;
    }
    // Get the setup params
    Eigen::Affine3d sdf_origin_transform = environment.GetOriginTransform();
    double sdf_cell_size = environment.GetResolution();
    double sdf_x_size = environment.GetXSize();
    double sdf_y_size = environment.GetYSize();
    double sdf_z_size = environment.GetZSize();
    // Make the CollisionMapGrid container
    sdf_tools::COLLISION_CELL default_cm_cell;
    default_cm_cell.occupancy = 0.0;
    default_cm_cell.component = 0;
    sdf_tools::CollisionMapGrid collision_map(sdf_origin_transform, "deformable_ompl", sdf_cell_size, sdf_x_size, sdf_y_size, sdf_z_size, default_cm_cell);
    // Now, loop through the intermediate grid to produce the final lists of filled/free cells
    std::vector<VoxelGrid::GRID_INDEX> filled;
    std::vector<VoxelGrid::GRID_INDEX> free;
    for (int64_t x_index = 0; x_index < environment.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < environment.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < environment.GetNumZCells(); z_index++)
            {
                VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                const DVXL& stored = environment.GetImmutable(current_index).first;
                // Check if the cell belongs to an object
                // If there re no objects to use, we use them all
                if (objects_to_use.size() == 0)
                {
                    // Check if the cell is filled or free (and that it belongs to an object)
                    if (stored.sensitivity > 0.0 && stored.object_id > 0)
                    {
                        filled.push_back(current_index);
                        sdf_tools::COLLISION_CELL filled_cell;
                        filled_cell.occupancy = 1.0;
                        filled_cell.component = 0;
                        collision_map.Set(x_index, y_index, z_index, filled_cell);
                    }
                    else
                    {
                        free.push_back(current_index);
                        sdf_tools::COLLISION_CELL free_cell;
                        free_cell.occupancy = 0.0;
                        free_cell.component = 0;
                        collision_map.Set(x_index, y_index, z_index, free_cell);
                    }
                }
                else
                {
                    // Check if the cell is filled or free (and that is belongs to an object we want)
                    if (stored.sensitivity > 0.0 && object_map[stored.object_id] > 0)
                    {
                        filled.push_back(current_index);
                        sdf_tools::COLLISION_CELL filled_cell;
                        filled_cell.occupancy = 1.0;
                        filled_cell.component = 0;
                        collision_map.Set(x_index, y_index, z_index, filled_cell);
                    }
                    else
                    {
                        free.push_back(current_index);
                        sdf_tools::COLLISION_CELL free_cell;
                        free_cell.occupancy = 0.0;
                        free_cell.component = 0;
                        collision_map.Set(x_index, y_index, z_index, free_cell);
                    }
                }
            }
        }
    }
    // Build the distance fields
    DistanceField filled_distance_field = BuildDistanceField(filled, sdf_origin_transform, sdf_cell_size, sdf_x_size, sdf_y_size, sdf_z_size);
    DistanceField free_distance_field = BuildDistanceField(free, sdf_origin_transform, sdf_cell_size, sdf_x_size, sdf_y_size, sdf_z_size);
    // Make the SDF container
    sdf_tools::SignedDistanceField sdf(sdf_origin_transform, "deformable_ompl", sdf_cell_size, sdf_x_size, sdf_y_size, sdf_z_size, INFINITY);
    // Fill in the SDF
    for (int64_t x_index = 0; x_index < sdf.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < sdf.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < sdf.GetNumZCells(); z_index++)
            {
                double distance1 = sqrt(filled_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * sdf_cell_size;
                double distance2 = sqrt(free_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * sdf_cell_size;
                sdf.Set(x_index, y_index, z_index, (distance1 - distance2));
            }
        }
    }
    // Update the connected components
    collision_map.UpdateConnectedComponents();
    // Return them
    return std::pair<sdf_tools::SignedDistanceField, sdf_tools::CollisionMapGrid>(sdf, collision_map);
}

DVXLCostFn::DistanceField DVXLCostFn::BuildDistanceField(std::vector<VoxelGrid::GRID_INDEX>& points, Eigen::Affine3d origin_transform, double cell_size, double x_size, double y_size, double z_size)
{
    // Make the DistanceField container
    bucket_cell default_cell;
    default_cell.distance_square = INFINITY;
    DistanceField distance_field(origin_transform, cell_size, x_size, y_size, z_size, default_cell);
    // Compute maximum distance square
    long max_distance_square = (distance_field.GetNumXCells() * distance_field.GetNumXCells()) + (distance_field.GetNumYCells() * distance_field.GetNumYCells()) + (distance_field.GetNumZCells() * distance_field.GetNumZCells());
    // Make bucket queue
    std::vector<std::vector<bucket_cell>> bucket_queue(max_distance_square + 1);
    bucket_queue[0].reserve(points.size());
    // Set initial update direction
    int initial_update_direction = GetDirectionNumber(0, 0, 0);
    // Mark all points with distance zero and add to the bucket queue
    for (size_t index = 0; index < points.size(); index++)
    {
        std::pair<bucket_cell&, bool> query = distance_field.GetMutable(points[index].x, points[index].y, points[index].z);
        // Check to make sure the cell is inside the bounds of the SDF
        if (query.second)
        {
            // Check to see if the cell has been processed already
            if (query.first.distance_square != 0.0)
            {
                query.first.location[0] = points[index].x;
                query.first.location[1] = points[index].y;
                query.first.location[2] = points[index].z;
                query.first.closest_point[0] = points[index].x;
                query.first.closest_point[1] = points[index].y;
                query.first.closest_point[2] = points[index].z;
                query.first.distance_square = 0.0;
                query.first.update_direction = initial_update_direction;
                bucket_queue[0].push_back(query.first);
            }
            // If we have already processed this cell, there's no need to do it again
            else
            {
                continue;
            }
        }
        // If the point is outside the bounds of the SDF, skip
        else
        {
            continue;
        }
    }
    // Process the bucket queue
    std::vector<std::vector<std::vector<std::vector<int>>>> neighborhoods = MakeNeighborhoods();
    for (size_t bq_idx = 0; bq_idx < bucket_queue.size(); bq_idx++)
    {
        std::vector<bucket_cell>::iterator queue_itr = bucket_queue[bq_idx].begin();
        while (queue_itr != bucket_queue[bq_idx].end())
        {
            // Get the current location
            bucket_cell& cur_cell = *queue_itr;
            double x = cur_cell.location[0];
            double y = cur_cell.location[1];
            double z = cur_cell.location[2];
            // Pick the update direction
            int D = bq_idx;
            if (D > 1)
            {
                D = 1;
            }
            // Make sure the update direction is valid
            if (cur_cell.update_direction < 0 || cur_cell.update_direction > 26)
            {
                ++queue_itr;
                continue;
            }
            // Get the current neighborhood list
            std::vector<std::vector<int>>& neighborhood = neighborhoods[D][cur_cell.update_direction];
            // Update the distance from the neighboring cells
            for (size_t nh_idx = 0; nh_idx < neighborhood.size(); nh_idx++)
            {
                // Get the direction to check
                int dx = neighborhood[nh_idx][0];
                int dy = neighborhood[nh_idx][1];
                int dz = neighborhood[nh_idx][2];
                int nx = x + dx;
                int ny = y + dy;
                int nz = z + dz;
                std::pair<bucket_cell&, bool> neighbor_query = distance_field.GetMutable((int64_t)nx, (int64_t)ny, (int64_t)nz);
                if (!neighbor_query.second)
                {
                    // "Neighbor" is outside the bounds of the SDF
                    continue;
                }
                // Update the neighbor's distance based on the current
                int new_distance_square = ComputeDistanceSquared(nx, ny, nz, cur_cell.closest_point[0], cur_cell.closest_point[1], cur_cell.closest_point[2]);
                if (new_distance_square > max_distance_square)
                {
                    // Skip these cases
                    continue;
                }
                if (new_distance_square < neighbor_query.first.distance_square)
                {
                    // If the distance is better, time to update the neighbor
                    neighbor_query.first.distance_square = new_distance_square;
                    neighbor_query.first.closest_point[0] = cur_cell.closest_point[0];
                    neighbor_query.first.closest_point[1] = cur_cell.closest_point[1];
                    neighbor_query.first.closest_point[2] = cur_cell.closest_point[2];
                    neighbor_query.first.location[0] = nx;
                    neighbor_query.first.location[1] = ny;
                    neighbor_query.first.location[2] = nz;
                    neighbor_query.first.update_direction = GetDirectionNumber(dx, dy, dz);
                    // Add the neighbor into the bucket queue
                    bucket_queue[new_distance_square].push_back(neighbor_query.first);
                }
            }
            // Increment the queue iterator
            ++queue_itr;
        }
        // Clear the current queue now that we're done with it
        bucket_queue[bq_idx].clear();
    }
    return distance_field;
}

void DVXLCostFn::UpdateSurfacesWithIntersectingPoints(const DVXLGrid& other_grid, std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces, const bool verbose) const
{
    size_t removed = 0;
    size_t added = 0;
    for (int64_t x_index = 0; x_index < other_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < other_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < other_grid.GetNumZCells(); z_index++)
            {
                std::vector<double> raw_relative_other_location = other_grid.GridIndexToLocation(x_index, y_index, z_index);
                // Remove from the grid
                std::vector<int64_t> location_index = environment_.LocationToGridIndex(raw_relative_other_location[0], raw_relative_other_location[1], raw_relative_other_location[2]);
                if (location_index.size() == 3)
                {
                    int64_t x_index = location_index[0];
                    int64_t y_index = location_index[1];
                    int64_t z_index = location_index[2];
                    // First, check if the cell is occupied
                    DVXL current_dvxl = environment_.GetImmutable(x_index, y_index, z_index).first;
                    if (current_dvxl.sensitivity > 0.0 && current_dvxl.object_id > 0)
                    {
                        // Now, remove the cell from the surface
                        VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                        surfaces[current_dvxl.object_id][current_index] = -1;
                        removed++;
                        // Now, attempt to add the six neighbors as new surface cells
                        VoxelGrid::GRID_INDEX up_index(x_index, y_index, z_index + 1);
                        VoxelGrid::GRID_INDEX down_index(x_index, y_index, z_index - 1);
                        VoxelGrid::GRID_INDEX left_index(x_index, y_index + 1, z_index);
                        VoxelGrid::GRID_INDEX right_index(x_index, y_index - 1, z_index);
                        VoxelGrid::GRID_INDEX front_index(x_index + 1, y_index, z_index);
                        VoxelGrid::GRID_INDEX back_index(x_index - 1, y_index, z_index);
                        // Check each neighboring cell to see if it's the same object
                        if (environment_.GetImmutable(up_index.x, up_index.y, up_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][up_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][up_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(down_index.x, down_index.y, down_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][down_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][down_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(left_index.x, left_index.y, left_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][left_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][left_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(right_index.x, right_index.y, right_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][right_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][right_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(front_index.x, front_index.y, front_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][front_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][front_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(back_index.x, back_index.y, back_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][back_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][back_index] = 1;
                                added++;
                            }
                        }
                    }
                }
            }
        }
    }
    if (verbose)
    {
        std::cerr << "Removed " << removed << " surface cells, added " << added << " new surface cells" << std::endl;
    }
}

void DVXLCostFn::UpdateSurfacesWithIntersectingPoints(const DVXLGrid& other_grid, const Eigen::Affine3d base_transform, std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces, const bool verbose) const
{
    size_t removed = 0;
    size_t added = 0;
    for (int64_t x_index = 0; x_index < other_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < other_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < other_grid.GetNumZCells(); z_index++)
            {
                std::vector<double> raw_relative_other_location = other_grid.GridIndexToLocation(x_index, y_index, z_index);
                // Remove from the grid
                Eigen::Vector3d relative_other_location(raw_relative_other_location[0], raw_relative_other_location[1], raw_relative_other_location[2]);
                Eigen::Vector3d other_location = base_transform * relative_other_location;
                // Remove from the grid, along with all 26 neighbors
                std::vector<int64_t> location_index = environment_.LocationToGridIndex(other_location.x(), other_location.y(), other_location.z());
                if (location_index.size() == 3)
                {
                    int64_t x_index = location_index[0];
                    int64_t y_index = location_index[1];
                    int64_t z_index = location_index[2];
                    // First, check if the cell is occupied
                    DVXL current_dvxl = environment_.GetImmutable(x_index, y_index, z_index).first;
                    if (current_dvxl.sensitivity > 0.0 && current_dvxl.object_id > 0)
                    {
                        // Now, remove the cell from the surface
                        VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                        surfaces[current_dvxl.object_id][current_index] = -1;
                        removed++;
                        // Now, attempt to add the six neighbors as new surface cells
                        VoxelGrid::GRID_INDEX up_index(x_index, y_index, z_index + 1);
                        VoxelGrid::GRID_INDEX down_index(x_index, y_index, z_index - 1);
                        VoxelGrid::GRID_INDEX left_index(x_index, y_index + 1, z_index);
                        VoxelGrid::GRID_INDEX right_index(x_index, y_index - 1, z_index);
                        VoxelGrid::GRID_INDEX front_index(x_index + 1, y_index, z_index);
                        VoxelGrid::GRID_INDEX back_index(x_index - 1, y_index, z_index);
                        // Check each neighboring cell to see if it's the same object
                        if (environment_.GetImmutable(up_index.x, up_index.y, up_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][up_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][up_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(down_index.x, down_index.y, down_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][down_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][down_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(left_index.x, left_index.y, left_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][left_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][left_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(right_index.x, right_index.y, right_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][right_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][right_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(front_index.x, front_index.y, front_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][front_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][front_index] = 1;
                                added++;
                            }
                        }
                        if (environment_.GetImmutable(back_index.x, back_index.y, back_index.z).first.object_id == current_dvxl.object_id)
                        {
                            // Make sure we haven't already removed it
                            if (surfaces[current_dvxl.object_id][back_index] >= 0)
                            {
                                surfaces[current_dvxl.object_id][back_index] = 1;
                                added++;
                            }
                        }
                    }
                }
            }
        }
    }
    if (verbose)
    {
        std::cerr << "Removed " << removed << " surface cells, added " << added << " new surface cells" << std::endl;
    }
}

void DVXLCostFn::UpdateSurfacesWithIntersectingPoints(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces, const bool verbose) const
{
    size_t removed = 0;
    size_t added = 0;
    for (size_t idx = 0; idx < points.size(); idx++)
    {
        const Eigen::Vector3d& point_location = points[idx].first;
        // Remove from the grid
        std::vector<int64_t> location_index = environment_.LocationToGridIndex(point_location.x(), point_location.y(), point_location.z());
        if (location_index.size() == 3)
        {
            int64_t x_index = location_index[0];
            int64_t y_index = location_index[1];
            int64_t z_index = location_index[2];
            // First, check if the cell is occupied
            DVXL current_dvxl = environment_.GetImmutable(x_index, y_index, z_index).first;
            if (current_dvxl.sensitivity > 0.0 && current_dvxl.object_id > 0)
            {
                // Now, remove the cell from the surface
                VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                surfaces[current_dvxl.object_id][current_index] = -1;
                removed++;
                // Now, attempt to add the six neighbors as new surface cells
                VoxelGrid::GRID_INDEX up_index(x_index, y_index, z_index + 1);
                VoxelGrid::GRID_INDEX down_index(x_index, y_index, z_index - 1);
                VoxelGrid::GRID_INDEX left_index(x_index, y_index + 1, z_index);
                VoxelGrid::GRID_INDEX right_index(x_index, y_index - 1, z_index);
                VoxelGrid::GRID_INDEX front_index(x_index + 1, y_index, z_index);
                VoxelGrid::GRID_INDEX back_index(x_index - 1, y_index, z_index);
                // Check each neighboring cell to see if it's the same object
                if (environment_.GetImmutable(up_index.x, up_index.y, up_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][up_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][up_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(down_index.x, down_index.y, down_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][down_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][down_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(left_index.x, left_index.y, left_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][left_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][left_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(right_index.x, right_index.y, right_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][right_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][right_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(front_index.x, front_index.y, front_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][front_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][front_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(back_index.x, back_index.y, back_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][back_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][back_index] = 1;
                        added++;
                    }
                }
            }
        }
    }
    if (verbose)
    {
        std::cerr << "Removed " << removed << " surface cells, added " << added << " new surface cells" << std::endl;
    }
}

void DVXLCostFn::UpdateSurfacesWithIntersectingPoints(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const Eigen::Affine3d base_transform, std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces, const bool verbose) const
{
    size_t removed = 0;
    size_t added = 0;
    for (size_t idx = 0; idx < points.size(); idx++)
    {
        Eigen::Vector3d point_location = base_transform * points[idx].first;
        // Remove from the grid
        std::vector<int64_t> location_index = environment_.LocationToGridIndex(point_location.x(), point_location.y(), point_location.z());
        if (location_index.size() == 3)
        {
            int64_t x_index = location_index[0];
            int64_t y_index = location_index[1];
            int64_t z_index = location_index[2];
            // First, check if the cell is occupied
            DVXL current_dvxl = environment_.GetImmutable(x_index, y_index, z_index).first;
            if (current_dvxl.sensitivity > 0.0 && current_dvxl.object_id > 0)
            {
                // Now, remove the cell from the surface
                VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                surfaces[current_dvxl.object_id][current_index] = -1;
                removed++;
                // Now, attempt to add the six neighbors as new surface cells
                VoxelGrid::GRID_INDEX up_index(x_index, y_index, z_index + 1);
                VoxelGrid::GRID_INDEX down_index(x_index, y_index, z_index - 1);
                VoxelGrid::GRID_INDEX left_index(x_index, y_index + 1, z_index);
                VoxelGrid::GRID_INDEX right_index(x_index, y_index - 1, z_index);
                VoxelGrid::GRID_INDEX front_index(x_index + 1, y_index, z_index);
                VoxelGrid::GRID_INDEX back_index(x_index - 1, y_index, z_index);
                // Check each neighboring cell to see if it's the same object
                if (environment_.GetImmutable(up_index.x, up_index.y, up_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][up_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][up_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(down_index.x, down_index.y, down_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][down_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][down_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(left_index.x, left_index.y, left_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][left_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][left_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(right_index.x, right_index.y, right_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][right_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][right_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(front_index.x, front_index.y, front_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][front_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][front_index] = 1;
                        added++;
                    }
                }
                if (environment_.GetImmutable(back_index.x, back_index.y, back_index.z).first.object_id == current_dvxl.object_id)
                {
                    // Make sure we haven't already removed it
                    if (surfaces[current_dvxl.object_id][back_index] >= 0)
                    {
                        surfaces[current_dvxl.object_id][back_index] = 1;
                        added++;
                    }
                }
            }
        }
    }
    if (verbose)
    {
        std::cerr << "Removed " << removed << " surface cells, added " << added << " new surface cells" << std::endl;
    }
}

bool DVXLCostFn::CheckForPuncture(std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& object_surfaces, const bool verbose) const
{
    // Compute the number of holes in each surface
    std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>::iterator object_surfaces_itr;
    for (object_surfaces_itr = object_surfaces.begin(); object_surfaces_itr != object_surfaces.end(); ++object_surfaces_itr)
    {
        u_int32_t object_number = object_surfaces_itr->first;
        std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& component_surface = object_surfaces_itr->second;
        std::pair<int32_t, int32_t> number_of_holes_and_voids = environment_.ComputeHolesInSurface(object_number, component_surface, verbose);
        int32_t number_of_holes = number_of_holes_and_voids.first;
        int32_t number_of_voids = number_of_holes_and_voids.second;
        if (number_of_holes > 0 || number_of_voids > 0)
        {
            return true;
        }
    }
    return false;
}

bool DVXLCostFn::CheckForPuncture(std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& current_object_surfaces, const std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& old_object_surfaces, const bool verbose) const
{
    // Compute the number of holes in each surface
    // Check the current surfaces against the old surfaces and ignore unchanged surfaces
    std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>::iterator object_surfaces_itr;
    for (object_surfaces_itr = current_object_surfaces.begin(); object_surfaces_itr != current_object_surfaces.end(); ++object_surfaces_itr)
    {
        u_int32_t object_number = object_surfaces_itr->first;
        std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& component_surface = object_surfaces_itr->second;
        // Get the old surfaces
        std::map<u_int32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>::const_iterator old_surface_query = old_object_surfaces.find(object_number);
        // Check if the current object isn't in the old surfaces - this means we check the surface
        if (old_surface_query == old_object_surfaces.end())
        {
            std::pair<int32_t, int32_t> number_of_holes_and_voids = environment_.ComputeHolesInSurface(object_number, component_surface, verbose);
            int32_t number_of_holes = number_of_holes_and_voids.first;
            int32_t number_of_voids = number_of_holes_and_voids.second;
            if (number_of_holes > 0 || number_of_voids > 0)
            {
                return true;
            }
        }
        // If the surface is there, check the size - we assume that surfaces only get larger, and can never get smaller
        else
        {
            // If the surfaces are different sizes, we check the surface
            if (current_object_surfaces.size() != old_surface_query->second.size())
            {
                std::pair<int32_t, int32_t> number_of_holes_and_voids = environment_.ComputeHolesInSurface(object_number, component_surface, verbose);
                int32_t number_of_holes = number_of_holes_and_voids.first;
                int32_t number_of_voids = number_of_holes_and_voids.second;
                if (number_of_holes > 0 || number_of_voids > 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool location_distance_pair_compare_fn(std::pair<Eigen::Vector3d, float> first, std::pair<Eigen::Vector3d, float> second)
{
    return first.second < second.second;
}

std::vector<Eigen::Vector3d> DVXLCostFn::ExtractLocalMinima(const double closeness_threshold) const
{
    // Generate all candidate local minima
    std::vector<std::pair<Eigen::Vector3d, float>> candidate_local_minima;
    for (int64_t x_idx = 0; x_idx < sdf_.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < sdf_.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < sdf_.GetNumZCells(); z_idx++)
            {
                // Check if the current index is a local minima
                if (IsLocalMinima(x_idx, y_idx, z_idx))
                {
                    // Convert to real world position
                    std::vector<double> raw_location = sdf_.GridIndexToLocation(x_idx, y_idx, z_idx);
                    if (raw_location.size() == 3)
                    {
                        Eigen::Vector3d location(raw_location[0], raw_location[1], raw_location[2]);
                        float distance = sdf_.Get(x_idx, y_idx, z_idx);
                        candidate_local_minima.push_back(std::pair<Eigen::Vector3d, float>(location, distance));
                    }
                }
            }
        }
    }
    // Iteratively extract the "best" local minima and remove all with the same distance
    std::vector<Eigen::Vector3d> local_minima;
    while (candidate_local_minima.size() > 0)
    {
        // Get the current best local minima
        std::pair<Eigen::Vector3d, float> best_local_minima = *std::min_element(candidate_local_minima.begin(), candidate_local_minima.end(), location_distance_pair_compare_fn);
        // Store it
        local_minima.push_back(best_local_minima.first);
        // Remove every canditate local minima with the same distance (actually, insert other ones into a new vector, then copy over)
        std::vector<std::pair<Eigen::Vector3d, float>> new_candidate_local_minima;
        for (size_t idx = 0; idx < candidate_local_minima.size(); idx++)
        {
            // If it's not close, keep it
            if (!CLOSE(candidate_local_minima[idx].second, best_local_minima.second))
            {
                new_candidate_local_minima.push_back(candidate_local_minima[idx]);
            }
            else
            {
                // If they have the same distance, but are far away
                if ((candidate_local_minima[idx].first - best_local_minima.first).norm() > (closeness_threshold))
                {
                    new_candidate_local_minima.push_back(candidate_local_minima[idx]);
                }
            }
        }
        // Update the candidate local minima
        candidate_local_minima = new_candidate_local_minima;
    }
    return local_minima;
}

std::vector<Eigen::Vector3d> DVXLCostFn::ExtractLocalMaxima(const double closeness_threshold) const
{
    std::vector<std::pair<Eigen::Vector3d, float>> candidate_local_maxima;
    for (int64_t x_idx = 0; x_idx < sdf_.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < sdf_.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < sdf_.GetNumZCells(); z_idx++)
            {
                // Check if the current index is a local minima
                if (IsLocalMaxima(x_idx, y_idx, z_idx))
                {
                    // Convert to real world position
                    std::vector<double> raw_location = sdf_.GridIndexToLocation(x_idx, y_idx, z_idx);
                    if (raw_location.size() == 3)
                    {
                        Eigen::Vector3d location(raw_location[0], raw_location[1], raw_location[2]);
                        float distance = sdf_.Get(x_idx, y_idx, z_idx);
                        candidate_local_maxima.push_back(std::pair<Eigen::Vector3d, float>(location, distance));
                    }
                }
            }
        }
    }
    // Iteratively extract the "best" local minima and remove all with the same distance
    std::vector<Eigen::Vector3d> local_maxima;
    while (candidate_local_maxima.size() > 0)
    {
        // Get the current best local maxima
        std::pair<Eigen::Vector3d, float> best_local_maxima = *std::max_element(candidate_local_maxima.begin(), candidate_local_maxima.end(), location_distance_pair_compare_fn);
        // Store it
        local_maxima.push_back(best_local_maxima.first);
        // Remove every canditate local maxima with the same distance (actually, insert other ones into a new vector, then copy over)
        std::vector<std::pair<Eigen::Vector3d, float>> new_candidate_local_maxima;
        for (size_t idx = 0; idx < candidate_local_maxima.size(); idx++)
        {
            // If it's not close, keep it
            if (!CLOSE(candidate_local_maxima[idx].second, best_local_maxima.second))
            {
                new_candidate_local_maxima.push_back(candidate_local_maxima[idx]);
            }
            else
            {
                // If they have the same distance, but are far away
                if ((candidate_local_maxima[idx].first - best_local_maxima.first).norm() > (closeness_threshold))
                {
                    new_candidate_local_maxima.push_back(candidate_local_maxima[idx]);
                }
            }
        }
        // Update the candidate local maxima
        candidate_local_maxima = new_candidate_local_maxima;
    }
    return local_maxima;
}
