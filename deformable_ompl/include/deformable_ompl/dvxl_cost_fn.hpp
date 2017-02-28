#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include "sdf_tools/sdf.hpp"
#include "sdf_tools/collision_map.hpp"
#include "deformable_ompl/dvxl_grid.hpp"

#ifndef DVXL_COST_FN_HPP
#define DVXL_COST_FN_HPP

namespace deformable_ompl
{
    typedef struct
    {
        Eigen::Vector3d position;
        Eigen::Vector3d gradient;
        uint32_t component;
        float penetration;
    } COLLISION_INFO;

    class DVXLCostFn
    {
    protected:

        bool initialized_;
        sdf_tools::SignedDistanceField sdf_;
        sdf_tools::CollisionMapGrid collision_map_;
        std::map<uint32_t, sdf_tools::SignedDistanceField> object_sdfs_;
        DVXLGrid environment_;

        inline double DVXLCost(const DVXL& dvxl1, const DVXL& dvxl2) const
        {
            if (dvxl1.deformability == 0.0 && dvxl2.deformability == 0.0)
            {
                return NAN;
            }
            else if (dvxl1.sensitivity == 0.0 || dvxl2.sensitivity == 0.0)
            {
                return 0.0;
            }
            else
            {
                double dvxl1_cost = (dvxl1.deformability / (dvxl1.deformability + dvxl2.deformability)) * dvxl1.sensitivity;
                double dvxl2_cost = (dvxl2.deformability / (dvxl1.deformability + dvxl2.deformability)) * dvxl2.sensitivity;
                return (dvxl1_cost + dvxl2_cost);
            }
        }

        inline bool CLOSE(const float first, const float second) const
        {
            if (fabs(first - second) < 0.001)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        inline Eigen::Vector3d MakeEigenVector3d(const std::vector<double>& position) const
        {
            return Eigen::Vector3d(position[0], position[1], position[2]);
        }

        inline bool IsLocalMinima(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            // Get the index
            float current = sdf_.Get(x_index, y_index, z_index);
            // Get the 26 neighbors
            std::vector<float> neighbors(26);
            neighbors[0] = sdf_.Get(x_index - 1, y_index - 1, z_index - 1);
            neighbors[1] = sdf_.Get(x_index - 1, y_index - 1, z_index + 0);
            neighbors[2] = sdf_.Get(x_index - 1, y_index - 1, z_index + 1);
            neighbors[3] = sdf_.Get(x_index - 1, y_index + 0, z_index - 1);
            neighbors[4] = sdf_.Get(x_index - 1, y_index + 0, z_index + 0);
            neighbors[5] = sdf_.Get(x_index - 1, y_index + 0, z_index + 1);
            neighbors[6] = sdf_.Get(x_index - 1, y_index + 1, z_index - 1);
            neighbors[7] = sdf_.Get(x_index - 1, y_index + 1, z_index + 0);
            neighbors[8] = sdf_.Get(x_index - 1, y_index + 1, z_index + 1);
            neighbors[9] = sdf_.Get(x_index + 0, y_index - 1, z_index - 1);
            neighbors[10] = sdf_.Get(x_index + 0, y_index - 1, z_index + 0);
            neighbors[11] = sdf_.Get(x_index + 0, y_index - 1, z_index + 1);
            neighbors[12] = sdf_.Get(x_index + 0, y_index + 0, z_index - 1);
            neighbors[13] = sdf_.Get(x_index + 0, y_index + 0, z_index + 1);
            neighbors[14] = sdf_.Get(x_index + 0, y_index + 1, z_index - 1);
            neighbors[15] = sdf_.Get(x_index + 0, y_index + 1, z_index + 0);
            neighbors[16] = sdf_.Get(x_index + 0, y_index + 1, z_index + 1);
            neighbors[17] = sdf_.Get(x_index + 1, y_index - 1, z_index - 1);
            neighbors[18] = sdf_.Get(x_index + 1, y_index - 1, z_index + 0);
            neighbors[19] = sdf_.Get(x_index + 1, y_index - 1, z_index + 1);
            neighbors[20] = sdf_.Get(x_index + 1, y_index + 0, z_index - 1);
            neighbors[21] = sdf_.Get(x_index + 1, y_index + 0, z_index + 0);
            neighbors[22] = sdf_.Get(x_index + 1, y_index + 0, z_index + 1);
            neighbors[23] = sdf_.Get(x_index + 1, y_index + 1, z_index - 1);
            neighbors[24] = sdf_.Get(x_index + 1, y_index + 1, z_index + 0);
            neighbors[25] = sdf_.Get(x_index + 1, y_index + 1, z_index + 1);
            // If the point is equal or less than all neighbors, it is a local minima
            if (current <= *std::min_element(neighbors.begin(), neighbors.end()))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        inline bool IsLocalMaxima(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            // Get the index
            float current = sdf_.Get(x_index, y_index, z_index);
            // Get the 26 neighbors
            std::vector<float> neighbors(26);
            neighbors[0] = sdf_.Get(x_index - 1, y_index - 1, z_index - 1);
            neighbors[1] = sdf_.Get(x_index - 1, y_index - 1, z_index + 0);
            neighbors[2] = sdf_.Get(x_index - 1, y_index - 1, z_index + 1);
            neighbors[3] = sdf_.Get(x_index - 1, y_index + 0, z_index - 1);
            neighbors[4] = sdf_.Get(x_index - 1, y_index + 0, z_index + 0);
            neighbors[5] = sdf_.Get(x_index - 1, y_index + 0, z_index + 1);
            neighbors[6] = sdf_.Get(x_index - 1, y_index + 1, z_index - 1);
            neighbors[7] = sdf_.Get(x_index - 1, y_index + 1, z_index + 0);
            neighbors[8] = sdf_.Get(x_index - 1, y_index + 1, z_index + 1);
            neighbors[9] = sdf_.Get(x_index + 0, y_index - 1, z_index - 1);
            neighbors[10] = sdf_.Get(x_index + 0, y_index - 1, z_index + 0);
            neighbors[11] = sdf_.Get(x_index + 0, y_index - 1, z_index + 1);
            neighbors[12] = sdf_.Get(x_index + 0, y_index + 0, z_index - 1);
            neighbors[13] = sdf_.Get(x_index + 0, y_index + 0, z_index + 1);
            neighbors[14] = sdf_.Get(x_index + 0, y_index + 1, z_index - 1);
            neighbors[15] = sdf_.Get(x_index + 0, y_index + 1, z_index + 0);
            neighbors[16] = sdf_.Get(x_index + 0, y_index + 1, z_index + 1);
            neighbors[17] = sdf_.Get(x_index + 1, y_index - 1, z_index - 1);
            neighbors[18] = sdf_.Get(x_index + 1, y_index - 1, z_index + 0);
            neighbors[19] = sdf_.Get(x_index + 1, y_index - 1, z_index + 1);
            neighbors[20] = sdf_.Get(x_index + 1, y_index + 0, z_index - 1);
            neighbors[21] = sdf_.Get(x_index + 1, y_index + 0, z_index + 0);
            neighbors[22] = sdf_.Get(x_index + 1, y_index + 0, z_index + 1);
            neighbors[23] = sdf_.Get(x_index + 1, y_index + 1, z_index - 1);
            neighbors[24] = sdf_.Get(x_index + 1, y_index + 1, z_index + 0);
            neighbors[25] = sdf_.Get(x_index + 1, y_index + 1, z_index + 1);
            // If the point is equal or greater than all neighbors, it is a local maxima
            if (current >= *std::max_element(neighbors.begin(), neighbors.end()))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        typedef struct
        {
            int64_t x_index;
            int64_t y_index;
            int64_t z_index;
        } CELL_INDEX;

        typedef struct
        {
            uint32_t location[3];
            uint32_t closest_point[3];
            double distance_square;
            int32_t update_direction;
        } bucket_cell;

        typedef VoxelGrid::VoxelGrid<bucket_cell> DistanceField;

        inline int GetDirectionNumber(int dx, int dy, int dz)
        {
            return ((dx + 1) * 9) + ((dy + 1) * 3) + (dz + 1);
        }

        inline double ComputeDistanceSquared(int32_t x1, int32_t y1, int32_t z1, int32_t x2, int32_t y2, int32_t z2)
        {
            int32_t dx = x1 - x2;
            int32_t dy = y1 - y2;
            int32_t dz = z1 - z2;
            return double((dx * dx) + (dy * dy) + (dz * dz));
        }

        inline std::vector<std::vector<std::vector<std::vector<int>>>> MakeNeighborhoods()
        {
            std::vector<std::vector<std::vector<std::vector<int>>>> neighborhoods;
            neighborhoods.resize(2);
            for (size_t n = 0; n < neighborhoods.size(); n++)
            {
                neighborhoods[n].resize(27);
                // Loop through the source directions
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            int direction_number = GetDirectionNumber(dx, dy, dz);
                            // Loop through the target directions
                            for (int tdx = -1; tdx <= 1; tdx++)
                            {
                                for (int tdy = -1; tdy <= 1; tdy++)
                                {
                                    for (int tdz = -1; tdz <= 1; tdz++)
                                    {
                                        if (tdx == 0 && tdy == 0 && tdz == 0)
                                        {
                                            continue;
                                        }
                                        if (n >= 1)
                                        {
                                            if ((abs(tdx) + abs(tdy) + abs(tdz)) != 1)
                                            {
                                                continue;
                                            }
                                            if ((dx * tdx) < 0 || (dy * tdy) < 0 || (dz * tdz) < 0)
                                            {
                                                continue;
                                            }
                                        }
                                        std::vector<int> new_point;
                                        new_point.resize(3);
                                        new_point[0] = tdx;
                                        new_point[1] = tdy;
                                        new_point[2] = tdz;
                                        neighborhoods[n][direction_number].push_back(new_point);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return neighborhoods;
        }

        std::vector<uint32_t> GetAllObjectIDs(const DVXLGrid& environment);

        DistanceField BuildDistanceField(std::vector<VoxelGrid::GRID_INDEX>& points, Eigen::Affine3d origin_transform, double cell_size, double x_size, double y_size, double z_size);

        std::pair<sdf_tools::SignedDistanceField, sdf_tools::CollisionMapGrid> UpdateInternalGrids(DVXLGrid& environment, std::vector<uint32_t>& objects_to_use);

    public:

        DVXLCostFn(std::string environment_filepath);

        DVXLCostFn(DVXLGrid environment);

        DVXLCostFn() : initialized_(false) {}

        inline const sdf_tools::SignedDistanceField& GetSDF() const
        {
            return sdf_;
        }

        inline sdf_tools::SignedDistanceField& GetMutableSDF()
        {
            return sdf_;
        }

        inline sdf_tools::SignedDistanceField CopySDF() const
        {
            return sdf_;
        }

        inline const sdf_tools::CollisionMapGrid& GetCollisionMapGrid() const
        {
            return collision_map_;
        }

        inline sdf_tools::CollisionMapGrid& GetMutableCollisionMapGrid()
        {
            return collision_map_;
        }

        inline sdf_tools::CollisionMapGrid CopyCollisionMapGrid() const
        {
            return collision_map_;
        }

        inline const DVXLGrid& GetDVXLGrid() const
        {
            return environment_;
        }

        inline DVXLGrid& GetMutableDVXLGrid()
        {
            return environment_;
        }

        inline DVXLGrid CopyDVXLGrid() const
        {
            return environment_;
        }

        std::vector<Eigen::Vector3d> ExtractMinimumVoronoiPoints(const uint32_t first_object_id, const uint32_t second_object_id) const;

        std::vector<Eigen::Vector3d> ExtractLocalMinima(const double closeness_threshold) const;

        std::vector<Eigen::Vector3d> ExtractLocalMaxima(const double closeness_threshold) const;

        double ComputeIntersectionCost(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points) const;

        double ComputeIntersectionCost(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const Eigen::Affine3d base_transform) const;

        double ComputeIntersectionCost(const DVXLGrid& other_grid) const;

        double ComputeIntersectionCost(const DVXLGrid& other_grid, const Eigen::Affine3d base_transform) const;

        void RemovePoints(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const bool remove_neighbors);

        void RemovePoints(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const Eigen::Affine3d base_transform, const bool remove_neighbors);

        void RemovePoints(const DVXLGrid& other_grid, const bool remove_neighbors);

        void RemovePoints(const DVXLGrid& other_grid, const Eigen::Affine3d base_transform, const bool remove_neighbors);

        std::map<uint32_t, std::vector<COLLISION_INFO>> GetCollisionInfo(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points) const;

        std::map<uint32_t, std::vector<COLLISION_INFO>> GetCollisionInfo(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const Eigen::Affine3d base_transform) const;

        std::map<uint32_t, std::vector<COLLISION_INFO>> GetCollisionInfo(const DVXLGrid& other_grid) const;

        std::map<uint32_t, std::vector<COLLISION_INFO>> GetCollisionInfo(const DVXLGrid& other_grid, const Eigen::Affine3d base_transform) const;

        void UpdateSurfacesWithIntersectingPoints(const DVXLGrid& other_grid, std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces, const bool verbose) const;

        void UpdateSurfacesWithIntersectingPoints(const DVXLGrid& other_grid, const Eigen::Affine3d base_transform, std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces, const bool verbose) const;

        void UpdateSurfacesWithIntersectingPoints(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces, const bool verbose) const;

        void UpdateSurfacesWithIntersectingPoints(const std::vector<std::pair<Eigen::Vector3d, DVXL>>& points, const Eigen::Affine3d base_transform, std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces, const bool verbose) const;

        bool CheckForPuncture(std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& object_surfaces, const bool verbose) const;

        bool CheckForPuncture(std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& current_object_surfaces, const std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& old_object_surfaces, const bool verbose) const;
    };
}

#endif // DVXL_COST_FN_HPP
