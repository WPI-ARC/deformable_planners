#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include "arc_utilities/voxel_grid.hpp"
#include "deformable_ompl/DvxlGrid.h"

#ifndef DVXL_GRID_HPP
#define DVXL_GRID_HPP

namespace deformable_ompl
{

    struct DVXL
    {
        float deformability;
        float sensitivity;
        float mass;
        float r;
        float g;
        float b;
        float a;
        uint32_t object_id;
    };

    inline std::vector<uint8_t> DVXLToBinary(DVXL value)
    {
        std::vector<uint8_t> binary(32);
        // Copy the floats into binary values
        uint32_t deformability_binary_value = 0;
        memcpy(&deformability_binary_value, &value.deformability, sizeof(uint32_t));
        uint32_t sensitivity_binary_value = 0;
        memcpy(&sensitivity_binary_value, &value.sensitivity, sizeof(uint32_t));
        uint32_t mass_binary_value = 0;
        memcpy(&mass_binary_value, &value.mass, sizeof(uint32_t));
        uint32_t r_binary_value = 0;
        memcpy(&r_binary_value, &value.r, sizeof(uint32_t));
        uint32_t g_binary_value = 0;
        memcpy(&g_binary_value, &value.g, sizeof(uint32_t));
        uint32_t b_binary_value = 0;
        memcpy(&b_binary_value, &value.b, sizeof(uint32_t));
        uint32_t a_binary_value = 0;
        memcpy(&a_binary_value, &value.a, sizeof(uint32_t));
        // Pack the binary values into a vector of bytes
        // Pack deformability
        // Copy byte 1, least-significant byte
        binary[3] = deformability_binary_value & 0x000000ff;
        // Copy byte 2
        deformability_binary_value = deformability_binary_value >> 8;
        binary[2] = deformability_binary_value & 0x000000ff;
        // Copy byte 3
        deformability_binary_value = deformability_binary_value >> 8;
        binary[1] = deformability_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        deformability_binary_value = deformability_binary_value >> 8;
        binary[0] = deformability_binary_value & 0x000000ff;
        // Pack sensitivity
        // Copy byte 1, least-significant byte
        binary[7] = sensitivity_binary_value & 0x000000ff;
        // Copy byte 2
        sensitivity_binary_value = sensitivity_binary_value >> 8;
        binary[6] = sensitivity_binary_value & 0x000000ff;
        // Copy byte 3
        sensitivity_binary_value = sensitivity_binary_value >> 8;
        binary[5] = sensitivity_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        sensitivity_binary_value = sensitivity_binary_value >> 8;
        binary[4] = sensitivity_binary_value & 0x000000ff;
        // Pack mass
        // Copy byte 1, least-significant byte
        binary[11] = mass_binary_value & 0x000000ff;
        // Copy byte 2
        mass_binary_value = mass_binary_value >> 8;
        binary[10] = mass_binary_value & 0x000000ff;
        // Copy byte 3
        mass_binary_value = mass_binary_value >> 8;
        binary[9] = mass_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        mass_binary_value = mass_binary_value >> 8;
        binary[8] = mass_binary_value & 0x000000ff;
        // Pack r
        // Copy byte 1, least-significant byte
        binary[15] = r_binary_value & 0x000000ff;
        // Copy byte 2
        r_binary_value = r_binary_value >> 8;
        binary[14] = r_binary_value & 0x000000ff;
        // Copy byte 3
        r_binary_value = r_binary_value >> 8;
        binary[13] = r_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        r_binary_value = r_binary_value >> 8;
        binary[12] = r_binary_value & 0x000000ff;
        // Pack g
        // Copy byte 1, least-significant byte
        binary[19] = g_binary_value & 0x000000ff;
        // Copy byte 2
        g_binary_value = g_binary_value >> 8;
        binary[18] = g_binary_value & 0x000000ff;
        // Copy byte 3
        g_binary_value = g_binary_value >> 8;
        binary[17] = g_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        g_binary_value = g_binary_value >> 8;
        binary[16] = g_binary_value & 0x000000ff;
        // Pack b
        // Copy byte 1, least-significant byte
        binary[23] = b_binary_value & 0x000000ff;
        // Copy byte 2
        b_binary_value = b_binary_value >> 8;
        binary[22] = b_binary_value & 0x000000ff;
        // Copy byte 3
        b_binary_value = b_binary_value >> 8;
        binary[21] = b_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        b_binary_value = b_binary_value >> 8;
        binary[20] = b_binary_value & 0x000000ff;
        // Pack a
        // Copy byte 1, least-significant byte
        binary[27] = a_binary_value & 0x000000ff;
        // Copy byte 2
        a_binary_value = a_binary_value >> 8;
        binary[26] = a_binary_value & 0x000000ff;
        // Copy byte 3
        a_binary_value = a_binary_value >> 8;
        binary[25] = a_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        a_binary_value = a_binary_value >> 8;
        binary[24] = a_binary_value & 0x000000ff;
        uint32_t object_id_binary_value = value.object_id;
        // Pack object_id
        // Copy byte 1, least-significant byte
        binary[31] = object_id_binary_value & 0x000000ff;
        // Copy byte 2
        object_id_binary_value = object_id_binary_value >> 8;
        binary[30] = object_id_binary_value & 0x000000ff;
        // Copy byte 3
        object_id_binary_value = object_id_binary_value >> 8;
        binary[29] = object_id_binary_value & 0x000000ff;
        // Copy byte 4, most-significant byte
        object_id_binary_value = object_id_binary_value >> 8;
        binary[28] = object_id_binary_value & 0x000000ff;
        return binary;
    }

    inline DVXL DVXLFromBinary(std::vector<uint8_t>& binary)
    {
        if (binary.size() != 32)
        {
            std::cerr << "Binary value is not 32 bytes" << std::endl;
            DVXL error_cell;
            error_cell.deformability = NAN;
            error_cell.sensitivity = NAN;
            error_cell.mass = NAN;
            error_cell.r = 0.0;
            error_cell.g = 0.0;
            error_cell.b = 0.0;
            error_cell.a = 0.0;
            error_cell.object_id = 0;
            return error_cell;
        }
        else
        {
            DVXL loaded;
            // Copy the bytes into binary values
            uint32_t deformability_binary_value = 0;
            // Copy in byte 4, most-significant byte
            deformability_binary_value = deformability_binary_value | binary[0];
            deformability_binary_value = deformability_binary_value << 8;
            // Copy in byte 3
            deformability_binary_value = deformability_binary_value | binary[1];
            deformability_binary_value = deformability_binary_value << 8;
            // Copy in byte 2
            deformability_binary_value = deformability_binary_value | binary[2];
            deformability_binary_value = deformability_binary_value << 8;
            // Copy in byte 1, least-significant byte
            deformability_binary_value = deformability_binary_value | binary[3];
            uint32_t sensitivity_binary_value = 0;
            // Copy in byte 4, most-significant byte
            sensitivity_binary_value = sensitivity_binary_value | binary[4];
            sensitivity_binary_value = sensitivity_binary_value << 8;
            // Copy in byte 3
            sensitivity_binary_value = sensitivity_binary_value | binary[5];
            sensitivity_binary_value = sensitivity_binary_value << 8;
            // Copy in byte 2
            sensitivity_binary_value = sensitivity_binary_value | binary[6];
            sensitivity_binary_value = sensitivity_binary_value << 8;
            // Copy in byte 1, least-significant byte
            sensitivity_binary_value = sensitivity_binary_value | binary[7];
            uint32_t mass_binary_value = 0;
            // Copy in byte 4, most-significant byte
            mass_binary_value = mass_binary_value | binary[8];
            mass_binary_value = mass_binary_value << 8;
            // Copy in byte 3
            mass_binary_value = mass_binary_value | binary[9];
            mass_binary_value = mass_binary_value << 8;
            // Copy in byte 2
            mass_binary_value = mass_binary_value | binary[10];
            mass_binary_value = mass_binary_value << 8;
            // Copy in byte 1, least-significant byte
            mass_binary_value = mass_binary_value | binary[11];
            uint32_t r_binary_value = 0;
            // Copy in byte 4, most-significant byte
            r_binary_value = r_binary_value | binary[12];
            r_binary_value = r_binary_value << 8;
            // Copy in byte 3
            r_binary_value = r_binary_value | binary[13];
            r_binary_value = r_binary_value << 8;
            // Copy in byte 2
            r_binary_value = r_binary_value | binary[14];
            r_binary_value = r_binary_value << 8;
            // Copy in byte 1, least-significant byte
            r_binary_value = r_binary_value | binary[15];
            uint32_t g_binary_value = 0;
            // Copy in byte 4, most-significant byte
            g_binary_value = g_binary_value | binary[16];
            g_binary_value = g_binary_value << 8;
            // Copy in byte 3
            g_binary_value = g_binary_value | binary[17];
            g_binary_value = g_binary_value << 8;
            // Copy in byte 2
            g_binary_value = g_binary_value | binary[18];
            g_binary_value = g_binary_value << 8;
            // Copy in byte 1, least-significant byte
            g_binary_value = g_binary_value | binary[19];
            uint32_t b_binary_value = 0;
            // Copy in byte 4, most-significant byte
            b_binary_value = b_binary_value | binary[20];
            b_binary_value = b_binary_value << 8;
            // Copy in byte 3
            b_binary_value = b_binary_value | binary[21];
            b_binary_value = b_binary_value << 8;
            // Copy in byte 2
            b_binary_value = b_binary_value | binary[22];
            b_binary_value = b_binary_value << 8;
            // Copy in byte 1, least-significant byte
            b_binary_value = b_binary_value | binary[23];
            uint32_t a_binary_value = 0;
            // Copy in byte 4, most-significant byte
            a_binary_value = a_binary_value | binary[24];
            a_binary_value = a_binary_value << 8;
            // Copy in byte 3
            a_binary_value = a_binary_value | binary[25];
            a_binary_value = a_binary_value << 8;
            // Copy in byte 2
            a_binary_value = a_binary_value | binary[26];
            a_binary_value = a_binary_value << 8;
            // Copy in byte 1, least-significant byte
            a_binary_value = a_binary_value | binary[27];
            uint32_t object_id_binary_value = 0;
            // Copy in byte 4, most-significant byte
            object_id_binary_value = object_id_binary_value | binary[28];
            object_id_binary_value = object_id_binary_value << 8;
            // Copy in byte 3
            object_id_binary_value = object_id_binary_value | binary[29];
            object_id_binary_value = object_id_binary_value << 8;
            // Copy in byte 2
            object_id_binary_value = object_id_binary_value | binary[30];
            object_id_binary_value = object_id_binary_value << 8;
            // Copy in byte 1, least-significant byte
            object_id_binary_value = object_id_binary_value | binary[31];
            // Copy the binary values into the cell
            memcpy(&loaded.deformability, &deformability_binary_value, sizeof(float));
            memcpy(&loaded.sensitivity, &sensitivity_binary_value, sizeof(float));
            memcpy(&loaded.mass, &mass_binary_value, sizeof(float));
            memcpy(&loaded.r, &r_binary_value, sizeof(float));
            memcpy(&loaded.g, &g_binary_value, sizeof(float));
            memcpy(&loaded.b, &b_binary_value, sizeof(float));
            memcpy(&loaded.a, &a_binary_value, sizeof(float));
            memcpy(&loaded.object_id, &object_id_binary_value, sizeof(uint32_t));
            return loaded;
        }
    }

    class DVXLGrid
    {
    protected:

        bool initialized_;
        std::string frame_;
        VoxelGrid::VoxelGrid<DVXL> dvxl_grid_;

        inline bool IsSurfaceIndex(const VoxelGrid::GRID_INDEX& index)
        {
            return IsSurfaceIndex(index.x, index.y, index.z);
        }

        inline bool IsSurfaceIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            // First, we make sure that indices are within bounds
            // Out of bounds indices are NOT surface cells
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= GetNumXCells() || y_index >= GetNumYCells() || z_index >= GetNumZCells())
            {
                return false;
            }
            // Empty cells are automatically not surface cells
            uint32_t our_object_id = dvxl_grid_.GetImmutable(x_index, y_index, z_index).first.object_id;
            if (our_object_id == 0)
            {
                return false;
            }
            // Edge indices could automatically be surface cells
            if (x_index == 0 || y_index == 0 || z_index == 0 || x_index == (GetNumXCells() - 1) || y_index == (GetNumYCells() - 1) || z_index == (GetNumZCells()))
            {
                if (our_object_id != dvxl_grid_.GetOOBValue().object_id)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            // If the cell is inside the grid, we check the neighbors
            // Note that we must check all 6 neighbors
            // Check neighbor 1
            if (our_object_id != dvxl_grid_.GetImmutable(x_index, y_index, z_index - 1).first.object_id)
            {
                return true;
            }
            // Check neighbor 2
            else if (our_object_id != dvxl_grid_.GetImmutable(x_index, y_index, z_index + 1).first.object_id)
            {
                return true;
            }
            // Check neighbor 3
            else if (our_object_id != dvxl_grid_.GetImmutable(x_index, y_index - 1, z_index).first.object_id)
            {
                return true;
            }
            // Check neighbor 4
            else if (our_object_id != dvxl_grid_.GetImmutable(x_index, y_index + 1, z_index).first.object_id)
            {
                return true;
            }
            // Check neighbor 5
            else if (our_object_id != dvxl_grid_.GetImmutable(x_index - 1, y_index, z_index).first.object_id)
            {
                return true;
            }
            // Check neighbor 6
            else if (our_object_id != dvxl_grid_.GetImmutable(x_index + 1, y_index, z_index).first.object_id)
            {
                return true;
            }
            // If none of the faces are exposed, it's not a surface voxel
            return false;
        }

        std::vector<uint8_t> PackBinaryRepresentation(const std::vector<DVXL>& raw) const;

        std::vector<DVXL> UnpackBinaryRepresentation(std::vector<uint8_t>& packed);

        DVXL GetByIndexFromGridAndSurface(const int64_t x_index, const int64_t y_index, const int64_t z_index, const std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& surface) const;

    public:

        DVXLGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, DVXL default_value, DVXL oob_value);

        DVXLGrid(Eigen::Affine3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, DVXL default_value, DVXL oob_value);

        DVXLGrid() : initialized_(false) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline std::pair<const DVXL&, bool> GetImmutable(const double x, const double y, const double z) const
        {
            return dvxl_grid_.GetImmutable(x, y, z);
        }

        inline std::pair<DVXL&, bool> GetMutable(const double x, const double y, const double z)
        {
            return dvxl_grid_.GetMutable(x, y, z);
        }

        inline std::pair<const DVXL&, bool> GetImmutable(const VoxelGrid::GRID_INDEX& index) const
        {
            return dvxl_grid_.GetImmutable(index);
        }

        inline std::pair<DVXL&, bool> GetMutable(const VoxelGrid::GRID_INDEX& index)
        {
            return dvxl_grid_.GetMutable(index);
        }

        inline std::pair<const DVXL&, bool> GetImmutable(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return dvxl_grid_.GetImmutable(x_index, y_index, z_index);
        }

        inline std::pair<DVXL&, bool> GetMutable(const int64_t x_index, const int64_t y_index, const int64_t z_index)
        {
            return dvxl_grid_.GetMutable(x_index, y_index, z_index);
        }

        inline bool Set(const double x, const double y, const double z, DVXL& value)
        {
            return dvxl_grid_.SetValue(x, y, z, value);
        }

        inline bool Set(const int64_t x_index, const int64_t y_index, const int64_t z_index, DVXL& value)
        {
            return dvxl_grid_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool CheckInBounds(const double x, const double y, const double z) const
        {
            return dvxl_grid_.GetImmutable(x, y, z).second;
        }

        inline bool CheckInBounds(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            if (x_index >= 0 && y_index >= 0 && z_index >= 0 && x_index < GetNumXCells() && y_index < GetNumYCells() && z_index < GetNumZCells())
            {
                return true;
            }
            else
            {
                return true;
            }
        }

        inline double GetXSize() const
        {
            return dvxl_grid_.GetXSize();
        }

        inline double GetYSize() const
        {
            return dvxl_grid_.GetYSize();
        }

        inline double GetZSize() const
        {
            return dvxl_grid_.GetZSize();
        }

        inline double GetResolution() const
        {
            return dvxl_grid_.GetCellSizes()[0];
        }

        inline DVXL GetDefaultValue() const
        {
            return dvxl_grid_.GetDefaultValue();
        }

        inline DVXL GetOOBValue() const
        {
            return dvxl_grid_.GetOOBValue();
        }

        inline int64_t GetNumXCells() const
        {
            return dvxl_grid_.GetNumXCells();
        }

        inline int64_t GetNumYCells() const
        {
            return dvxl_grid_.GetNumYCells();
        }

        inline int64_t GetNumZCells() const
        {
            return dvxl_grid_.GetNumZCells();
        }

        inline Eigen::Affine3d GetOriginTransform() const
        {
            return dvxl_grid_.GetOriginTransform();
        }

        inline std::vector<int64_t> LocationToGridIndex(const double x, const double y, const double z) const
        {
            return dvxl_grid_.LocationToGridIndex(x, y, z);
        }

        inline std::vector<double> GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return dvxl_grid_.GridIndexToLocation(x_index, y_index, z_index);
        }

        bool SaveToFile(std::string& filepath);

        bool LoadFromFile(std::string& filepath);

        DvxlGrid GetMessageRepresentation();

        bool LoadFromMessageRepresentation(DvxlGrid& message);

        visualization_msgs::Marker ExportForDisplay() const;

        visualization_msgs::Marker ExportSurfaceForDisplay(const std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& surface) const;

        visualization_msgs::Marker ExportSurfacesForDisplay(const std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces) const;

        std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>> ExtractObjectSurfaces() const;

        std::pair<int32_t, int32_t> ComputeHolesInSurface(const uint32_t object_id, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> surface, const bool verbose) const;

        int32_t ComputeConnectivityOfSurfaceVertices(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface_vertices_connectivity) const;
    };
}

#endif // DVXL_GRID_HPP
