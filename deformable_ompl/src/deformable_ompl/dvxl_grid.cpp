#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>
#include <ros/ros.h>
#include <list>
#include <unordered_map>
#include "arc_utilities/zlib_helpers.hpp"
#include "deformable_ompl/dvxl_grid.hpp"

using namespace deformable_ompl;

DVXLGrid::DVXLGrid(std::string frame, double resolution, double x_size, double y_size, double z_size, DVXL default_value, DVXL OOB_value) : initialized_(true)
{
    frame_ = frame;
    VoxelGrid::VoxelGrid<DVXL> new_field(resolution, x_size, y_size, z_size, default_value, OOB_value);
    dvxl_grid_ = new_field;
}

DVXLGrid::DVXLGrid(Eigen::Affine3d origin_transform, std::string frame, double resolution, double x_size, double y_size, double z_size, DVXL default_value, DVXL OOB_value) : initialized_(true)
{
    frame_ = frame;
    VoxelGrid::VoxelGrid<DVXL> new_field(origin_transform, resolution, x_size, y_size, z_size, default_value, OOB_value);
    dvxl_grid_ = new_field;
}

bool DVXLGrid::SaveToFile(std::string& filepath)
{
    // Convert to message representation
    DvxlGrid message_rep = GetMessageRepresentation();
    // Save message to file
    try
    {
        std::ofstream output_file(filepath.c_str(), std::ios::out|std::ios::binary);
        uint32_t serialized_size = ros::serialization::serializationLength(message_rep);
        std::unique_ptr<uint8_t> ser_buffer(new uint8_t[serialized_size]);
        ros::serialization::OStream ser_stream(ser_buffer.get(), serialized_size);
        ros::serialization::serialize(ser_stream, message_rep);
        output_file.write((char*)ser_buffer.get(), serialized_size);
        output_file.close();
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool DVXLGrid::LoadFromFile(std::string& filepath)
{
    try
    {
        // Load message from file
        std::ifstream input_file(filepath.c_str(), std::ios::in|std::ios::binary);
        input_file.seekg(0, std::ios::end);
        std::streampos end = input_file.tellg();
        input_file.seekg(0, std::ios::beg);
        std::streampos begin = input_file.tellg();
        uint32_t serialized_size = end - begin;
        std::unique_ptr<uint8_t> deser_buffer(new uint8_t[serialized_size]);
        input_file.read((char*) deser_buffer.get(), serialized_size);
        ros::serialization::IStream deser_stream(deser_buffer.get(), serialized_size);
        DvxlGrid new_message;
        ros::serialization::deserialize(deser_stream, new_message);
        // Load state from the message
        bool success = LoadFromMessageRepresentation(new_message);
        return success;
    }
    catch (...)
    {
        return false;
    }
}

std::vector<uint8_t> DVXLGrid::PackBinaryRepresentation(const std::vector<DVXL>& raw) const
{
    std::vector<uint8_t> packed(raw.size() * 32);
    for (size_t field_idx = 0, binary_index = 0; field_idx < raw.size(); field_idx++, binary_index+=32)
    {
        const DVXL& raw_cell = raw[field_idx];
        std::vector<uint8_t> packed_cell = DVXLToBinary(raw_cell);
        packed[binary_index + 0] = packed_cell[0];
        packed[binary_index + 1] = packed_cell[1];
        packed[binary_index + 2] = packed_cell[2];
        packed[binary_index + 3] = packed_cell[3];
        packed[binary_index + 4] = packed_cell[4];
        packed[binary_index + 5] = packed_cell[5];
        packed[binary_index + 6] = packed_cell[6];
        packed[binary_index + 7] = packed_cell[7];
        packed[binary_index + 8] = packed_cell[8];
        packed[binary_index + 9] = packed_cell[9];
        packed[binary_index + 10] = packed_cell[10];
        packed[binary_index + 11] = packed_cell[11];
        packed[binary_index + 12] = packed_cell[12];
        packed[binary_index + 13] = packed_cell[13];
        packed[binary_index + 14] = packed_cell[14];
        packed[binary_index + 15] = packed_cell[15];
        packed[binary_index + 16] = packed_cell[16];
        packed[binary_index + 17] = packed_cell[17];
        packed[binary_index + 18] = packed_cell[18];
        packed[binary_index + 19] = packed_cell[19];
        packed[binary_index + 20] = packed_cell[20];
        packed[binary_index + 21] = packed_cell[21];
        packed[binary_index + 22] = packed_cell[22];
        packed[binary_index + 23] = packed_cell[23];
        packed[binary_index + 24] = packed_cell[24];
        packed[binary_index + 25] = packed_cell[25];
        packed[binary_index + 26] = packed_cell[26];
        packed[binary_index + 27] = packed_cell[27];
        packed[binary_index + 28] = packed_cell[28];
        packed[binary_index + 29] = packed_cell[29];
        packed[binary_index + 30] = packed_cell[30];
        packed[binary_index + 31] = packed_cell[31];
    }
    return packed;
}

std::vector<DVXL> DVXLGrid::UnpackBinaryRepresentation(std::vector<uint8_t>& packed)
{
    if ((packed.size() % 32) != 0)
    {
        std::cerr << "Invalid binary representation - length is not a multiple of 32" << std::endl;
        return std::vector<DVXL>();
    }
    uint64_t data_size = packed.size() / 32;
    std::vector<DVXL> unpacked(data_size);
    for (size_t field_idx = 0, binary_index = 0; field_idx < unpacked.size(); field_idx++, binary_index+=32)
    {
        std::vector<uint8_t> binary_block{packed[binary_index], packed[binary_index + 1], packed[binary_index + 2], packed[binary_index + 3], packed[binary_index + 4], packed[binary_index + 5], packed[binary_index + 6], packed[binary_index + 7], packed[binary_index + 8], packed[binary_index + 9], packed[binary_index + 10], packed[binary_index + 11], packed[binary_index + 12], packed[binary_index + 13], packed[binary_index + 14], packed[binary_index + 15], packed[binary_index + 16], packed[binary_index + 17], packed[binary_index + 18], packed[binary_index + 19], packed[binary_index + 20], packed[binary_index + 21], packed[binary_index + 22], packed[binary_index + 23], packed[binary_index + 24], packed[binary_index + 25], packed[binary_index + 26], packed[binary_index + 27], packed[binary_index + 28], packed[binary_index + 29], packed[binary_index + 30], packed[binary_index + 31]};
        unpacked[field_idx] = DVXLFromBinary(binary_block);
    }
    return unpacked;
}

DvxlGrid DVXLGrid::GetMessageRepresentation()
{
    DvxlGrid message_rep;
    // Populate message
    message_rep.header.frame_id = frame_;
    const Eigen::Affine3d& origin_transform = dvxl_grid_.GetOriginTransform();
    message_rep.origin_transform.translation.x = origin_transform.translation().x();
    message_rep.origin_transform.translation.y = origin_transform.translation().y();
    message_rep.origin_transform.translation.z = origin_transform.translation().z();
    const Eigen::Quaterniond origin_transform_rotation(origin_transform.rotation());
    message_rep.origin_transform.rotation.x = origin_transform_rotation.x();
    message_rep.origin_transform.rotation.y = origin_transform_rotation.y();
    message_rep.origin_transform.rotation.z = origin_transform_rotation.z();
    message_rep.origin_transform.rotation.w = origin_transform_rotation.w();
    message_rep.dimensions.x = dvxl_grid_.GetXSize();
    message_rep.dimensions.y = dvxl_grid_.GetYSize();
    message_rep.dimensions.z = dvxl_grid_.GetZSize();
    message_rep.dvxl_cell_size = GetResolution();
    DVXL default_dvxl = dvxl_grid_.GetDefaultValue();
    message_rep.OOB_deformability = default_dvxl.deformability;
    message_rep.OOB_sensitivity = default_dvxl.sensitivity;
    message_rep.OOB_mass = default_dvxl.mass;
    message_rep.OOB_r = default_dvxl.r;
    message_rep.OOB_g = default_dvxl.g;
    message_rep.OOB_b = default_dvxl.b;
    message_rep.OOB_a = default_dvxl.a;
    std::vector<DVXL> raw_data = dvxl_grid_.GetRawData();
    std::vector<uint8_t> binary_data = PackBinaryRepresentation(raw_data);
    message_rep.data = ZlibHelpers::CompressBytes(binary_data);
    return message_rep;
}

bool DVXLGrid::LoadFromMessageRepresentation(DvxlGrid& message)
{
    // Make a new voxel grid inside
    Eigen::Translation3d origin_translation(message.origin_transform.translation.x, message.origin_transform.translation.y, message.origin_transform.translation.z);
    Eigen::Quaterniond origin_rotation(message.origin_transform.rotation.w, message.origin_transform.rotation.x, message.origin_transform.rotation.y, message.origin_transform.rotation.z);
    Eigen::Affine3d origin_transform = origin_translation * origin_rotation;
    DVXL OOB_value;
    OOB_value.deformability = message.OOB_deformability;
    OOB_value.sensitivity = message.OOB_sensitivity;
    OOB_value.mass = message.OOB_mass;
    OOB_value.r = message.OOB_r;
    OOB_value.g = message.OOB_g;
    OOB_value.b = message.OOB_b;
    OOB_value.a = message.OOB_a;
    VoxelGrid::VoxelGrid<DVXL> new_field(origin_transform, message.dvxl_cell_size, message.dimensions.x, message.dimensions.y, message.dimensions.z, OOB_value);
    // Unpack the binary data
    std::vector<uint8_t> binary_representation = ZlibHelpers::DecompressBytes(message.data);
    std::vector<DVXL> unpacked = UnpackBinaryRepresentation(binary_representation);
    if (unpacked.empty())
    {
        std::cerr << "Unpack returned an empty DVXLGrid" << std::endl;
        return false;
    }
    bool success = new_field.SetRawData(unpacked);
    if (!success)
    {
        std::cerr << "Unable to set internal representation of the DVXLGrid" << std::endl;
        return false;
    }
    // Set it
    dvxl_grid_ = new_field;
    frame_ = message.header.frame_id;
    initialized_ = true;
    return true;
}

visualization_msgs::Marker DVXLGrid::ExportForDisplay() const
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "dvxl_grid";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = GetResolution();
    display_rep.scale.y = GetResolution();
    display_rep.scale.z = GetResolution();
    // Add all the cells of the DVXL grid to the message
    for (int64_t x_index = 0; x_index < dvxl_grid_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < dvxl_grid_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < dvxl_grid_.GetNumZCells(); z_index++)
            {
                DVXL dvxl_cell = dvxl_grid_.GetImmutable(x_index, y_index, z_index).first;
                if (dvxl_cell.a == 0.0 || dvxl_cell.sensitivity == 0.0)
                {
                    continue;
                }
                // Convert grid indices into a real-world location
                std::vector<double> location = dvxl_grid_.GridIndexToLocation(x_index, y_index, z_index);
                geometry_msgs::Point new_point;
                new_point.x = location[0];
                new_point.y = location[1];
                new_point.z = location[2];
                display_rep.points.push_back(new_point);
                std_msgs::ColorRGBA new_color;
                new_color.r = dvxl_cell.r;
                new_color.g = dvxl_cell.g;
                new_color.b = dvxl_cell.b;
                new_color.a = dvxl_cell.a;
                display_rep.colors.push_back(new_color);
            }
        }
    }
    return display_rep;
}

visualization_msgs::Marker DVXLGrid::ExportSurfaceForDisplay(const std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& surface) const
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "dvxl_grid_surface";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = GetResolution();
    display_rep.scale.y = GetResolution();
    display_rep.scale.z = GetResolution();
    // Add all the cells of the surface
    std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>::const_iterator surface_itr;
    for (surface_itr = surface.begin(); surface_itr != surface.end(); ++surface_itr)
    {
        VoxelGrid::GRID_INDEX index = surface_itr->first;
        int8_t validity = surface_itr->second;
        if (validity == 1)
        {
            DVXL dvxl_cell = dvxl_grid_.GetImmutable(index.x, index.y, index.z).first;
            if (dvxl_cell.a == 0.0 || dvxl_cell.sensitivity == 0.0)
            {
                std::cerr << "Surface contains cells that are not part of a visible object" << std::endl;
                continue;
            }
            // Convert grid indices into a real-world location
            std::vector<double> location = dvxl_grid_.GridIndexToLocation(index.x, index.y, index.z);
            geometry_msgs::Point new_point;
            new_point.x = location[0];
            new_point.y = location[1];
            new_point.z = location[2];
            display_rep.points.push_back(new_point);
            std_msgs::ColorRGBA new_color;
            new_color.r = dvxl_cell.r;
            new_color.g = dvxl_cell.g;
            new_color.b = dvxl_cell.b;
            new_color.a = dvxl_cell.a;
            display_rep.colors.push_back(new_color);
        }
    }
    return display_rep;
}

visualization_msgs::Marker DVXLGrid::ExportSurfacesForDisplay(const std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>& surfaces) const
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "dvxl_grid_surface";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = GetResolution();
    display_rep.scale.y = GetResolution();
    display_rep.scale.z = GetResolution();
    // Add all the cells of the surface
    std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>>::const_iterator surfaces_itr;
    for (surfaces_itr = surfaces.begin(); surfaces_itr != surfaces.end(); ++surfaces_itr)
    {
        const std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& surface = surfaces_itr->second;
        std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>::const_iterator surface_itr;
        for (surface_itr = surface.begin(); surface_itr != surface.end(); ++surface_itr)
        {
            VoxelGrid::GRID_INDEX index = surface_itr->first;
            int8_t validity = surface_itr->second;
            if (validity == 1)
            {
                DVXL dvxl_cell = dvxl_grid_.GetImmutable(index.x, index.y, index.z).first;
                if (dvxl_cell.a == 0.0 || dvxl_cell.sensitivity == 0.0)
                {
                    std::cerr << "Surface contains cells that are not part of a visible object" << std::endl;
                    continue;
                }
                // Convert grid indices into a real-world location
                std::vector<double> location = dvxl_grid_.GridIndexToLocation(index.x, index.y, index.z);
                geometry_msgs::Point new_point;
                new_point.x = location[0];
                new_point.y = location[1];
                new_point.z = location[2];
                display_rep.points.push_back(new_point);
                std_msgs::ColorRGBA new_color;
                new_color.r = dvxl_cell.r;
                new_color.g = dvxl_cell.g;
                new_color.b = dvxl_cell.b;
                new_color.a = dvxl_cell.a;
                display_rep.colors.push_back(new_color);
            }
        }
    }
    return display_rep;
}

std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>> DVXLGrid::ExtractObjectSurfaces() const
{
    std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>> object_surfaces;
    // Loop through the grid and extract surface cells for each component
    for (int64_t x_index = 0; x_index < dvxl_grid_.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < dvxl_grid_.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < dvxl_grid_.GetNumZCells(); z_index++)
            {
                const DVXL& current_cell = dvxl_grid_.GetImmutable(x_index, y_index, z_index).first;
                VoxelGrid::GRID_INDEX current_index(x_index, y_index, z_index);
                if (IsSurfaceIndex(x_index, y_index, z_index))
                {
                    object_surfaces[current_cell.object_id][current_index] = 1;
                }
            }
        }
    }
    return object_surfaces;
}

DVXL DVXLGrid::GetByIndexFromGridAndSurface(const int64_t x_index, const int64_t y_index, const int64_t z_index, const std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>& surface) const
{
    // Since we don't update the underlying grid with colliding points, we instead have to check if the
    // point has been removed via the object surface - i.e. if the surface value for the index is -1
    // First, check if the index is in the surface
    std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>::const_iterator index_in_surface = surface.find(VoxelGrid::GRID_INDEX(x_index, y_index, z_index));
    // If it's not in the surface, we look it up in the grid
    if (index_in_surface == surface.end())
    {
        DVXL in_grid = dvxl_grid_.GetImmutable(x_index, y_index, z_index).first;
        return in_grid;
    }
    // Check the surface cell
    else
    {
        int8_t stored_value = index_in_surface->second;
        // If the stored value is -1, then we've removed in the surface
        if (stored_value == -1)
        {
            DVXL empty;
            empty.deformability = 1.0;
            empty.sensitivity = 0.0;
            empty.mass = 0.0;
            empty.r = 0.0;
            empty.g = 0.0;
            empty.b = 0.0;
            empty.a = 0.0;
            empty.object_id = 0;
            return empty;
        }
        // If not, return the cell from the grid
        else
        {
            DVXL in_grid = dvxl_grid_.GetImmutable(x_index, y_index, z_index).first;
            return in_grid;
        }
    }
}

std::pair<int32_t, int32_t> DVXLGrid::ComputeHolesInSurface(const uint32_t object_id, std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> surface, const bool verbose) const
{
    // We have a list of all voxels with an exposed surface face
    // We loop through this list of voxels, and convert each voxel
    // into 8 vertices (the corners), which we individually check:
    //
    // First - we check to see if the vertex has already been
    // evaluated
    //
    // Second - we check if the vertex is actually on the surface
    // (make sure at least one of the three adjacent vertices is
    // exposed)
    //
    // Third - insert into hashtable of surface vertices
    //
    // Once we have completed this process, we loop back through
    // the hashtable of surface vertices and compute the number
    // of distance-1 neighboring surface vertices (we do this by
    // checking each of the six potential neighbor vertices) and
    // keep a running count of all vertices with 3, 5, and 6
    // neighbors.
    //
    // Once we have evaluated all the neighbors of all surface
    // vertices, we count the number of holes in the grid using
    // the formula from Chen and Rong, "Linear Time Recognition
    // Algorithms for Topological Invariants in 3D":
    //
    // #holes = 1 + (M5 + 2 * M6 - M3) / 8
    //
    // where M5 is the number of vertices with 5 neighbors,
    // M6 is the number of vertices with 6 neighbors, and
    // M3 is the number of vertices with 3 neighbors
    //
    // Storage for surface vertices
#ifdef ENABLE_UNORDERED_MAP_SIZE_HINTS
    size_t surface_vertices_size_hint = surface.size() * 8;
    std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> surface_vertices(surface_vertices_size_hint);
#else
    std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> surface_vertices;
#endif
    // Loop through all the surface voxels and extract surface vertices
    uint64_t surface_size = 0;
    std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>::const_iterator surface_itr;
    for (surface_itr = surface.begin(); surface_itr != surface.end(); ++surface_itr)
    {
        const VoxelGrid::GRID_INDEX& current_index = surface_itr->first;
        int8_t is_valid = surface_itr->second;
        // Since we can now have 'invalid' surface cells
        if (is_valid == 1)
        {
            // First, grab all six neighbors from the grid
            DVXL xyzm1 = GetByIndexFromGridAndSurface(current_index.x, current_index.y, current_index.z - 1, surface);
            DVXL xyzp1 = GetByIndexFromGridAndSurface(current_index.x, current_index.y, current_index.z + 1, surface);
            DVXL xym1z = GetByIndexFromGridAndSurface(current_index.x, current_index.y - 1, current_index.z, surface);
            DVXL xyp1z = GetByIndexFromGridAndSurface(current_index.x, current_index.y + 1, current_index.z, surface);
            DVXL xm1yz = GetByIndexFromGridAndSurface(current_index.x - 1, current_index.y, current_index.z, surface);
            DVXL xp1yz = GetByIndexFromGridAndSurface(current_index.x + 1, current_index.y, current_index.z, surface);
            // First, make sure it's actually connected by a face to a neighbor - since we aren't doing connected components first, this can occur
            // If all neighbors belong to different objects, we ignore this cell
            if (object_id != xyzm1.object_id && object_id != xyzp1.object_id && object_id != xym1z.object_id && object_id != xyp1z.object_id && object_id != xm1yz.object_id && object_id != xp1yz.object_id)
            {
                if (verbose)
                {
                    std::cerr << "Ignoring disconnected surface cell" << std::endl;
                }
                // Remove the cell from the surface
                surface[current_index] = -1;
                // Skip processing the cell
                continue;
            }
            // Generate all 8 vertices for the current voxel, check if an adjacent face of the current voxel is on the surface, and insert it if so
            // First, check the (-,-,-) vertex
            if (object_id != xyzm1.object_id || object_id != xym1z.object_id || object_id != xm1yz.object_id)
            {
                VoxelGrid::GRID_INDEX vertex1(current_index.x, current_index.y, current_index.z);
                surface_vertices[vertex1] = 1;
            }
            // Second, check the (-,-,+) vertex
            if (object_id != xyzp1.object_id || object_id != xym1z.object_id || object_id != xm1yz.object_id)
            {
                VoxelGrid::GRID_INDEX vertex2(current_index.x, current_index.y, current_index.z + 1);
                surface_vertices[vertex2] = 1;
            }
            // Third, check the (-,+,-) vertex
            if (object_id != xyzm1.object_id || object_id != xyp1z.object_id || object_id != xm1yz.object_id)
            {
                VoxelGrid::GRID_INDEX vertex3(current_index.x, current_index.y + 1, current_index.z);
                surface_vertices[vertex3] = 1;
            }
            // Fourth, check the (-,+,+) vertex
            if (object_id != xyzp1.object_id || object_id != xyp1z.object_id || object_id != xm1yz.object_id)
            {
                VoxelGrid::GRID_INDEX vertex4(current_index.x, current_index.y + 1, current_index.z + 1);
                surface_vertices[vertex4] = 1;
            }
            // Fifth, check the (+,-,-) vertex
            if (object_id != xyzm1.object_id || object_id != xym1z.object_id || object_id != xp1yz.object_id)
            {
                VoxelGrid::GRID_INDEX vertex5(current_index.x + 1, current_index.y, current_index.z);
                surface_vertices[vertex5] = 1;
            }
            // Sixth, check the (+,-,+) vertex
            if (object_id != xyzp1.object_id || object_id != xym1z.object_id || object_id != xp1yz.object_id)
            {
                VoxelGrid::GRID_INDEX vertex6(current_index.x + 1, current_index.y, current_index.z + 1);
                surface_vertices[vertex6] = 1;
            }
            // Seventh, check the (+,+,-) vertex
            if (object_id != xyzm1.object_id || object_id != xyp1z.object_id || object_id != xp1yz.object_id)
            {
                VoxelGrid::GRID_INDEX vertex7(current_index.x + 1, current_index.y + 1, current_index.z);
                surface_vertices[vertex7] = 1;
            }
            // Eighth, check the (+,+,+) vertex
            if (object_id != xyzp1.object_id || object_id != xyp1z.object_id || object_id != xp1yz.object_id)
            {
                VoxelGrid::GRID_INDEX vertex8(current_index.x + 1, current_index.y + 1, current_index.z + 1);
                surface_vertices[vertex8] = 1;
            }
            surface_size++;
        }
    }
    if (verbose)
    {
        std::cerr << "Surface with " << surface_size << " voxels has " << surface_vertices.size() << " surface vertices" << std::endl;
    }
    // Iterate through the surface vertices and count the neighbors of each vertex
    int32_t M3 = 0;
    int32_t M5 = 0;
    int32_t M6 = 0;
    // Store the connectivity of each vertex
    size_t vertex_connectivity_size_hint = surface_vertices.size();
    std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t> vertex_connectivity(vertex_connectivity_size_hint);
    std::unordered_map<VoxelGrid::GRID_INDEX, int8_t>::iterator surface_vertices_itr;
    for (surface_vertices_itr = surface_vertices.begin(); surface_vertices_itr != surface_vertices.end(); ++surface_vertices_itr)
    {
        VoxelGrid::GRID_INDEX key = surface_vertices_itr->first;
        VoxelGrid::GRID_INDEX value = key;
        // Insert into the connectivity map
        vertex_connectivity[key] = 0b00000000;
        // Check the six edges from the current vertex and count the number of exposed edges
        // (an edge is exposed if the at least one of the four surrounding voxels is not part
        // of the current component)
        int32_t edge_count = 0;
        // First, get the 8 voxels that surround the current vertex
        DVXL xm1ym1zm1 = GetByIndexFromGridAndSurface(value.x - 1, value.y - 1, value.z - 1, surface);
        DVXL xm1ym1zp1 = GetByIndexFromGridAndSurface(value.x - 1, value.y - 1, value.z + 0, surface);
        DVXL xm1yp1zm1 = GetByIndexFromGridAndSurface(value.x - 1, value.y + 0, value.z - 1, surface);
        DVXL xm1yp1zp1 = GetByIndexFromGridAndSurface(value.x - 1, value.y + 0, value.z + 0, surface);
        DVXL xp1ym1zm1 = GetByIndexFromGridAndSurface(value.x + 0, value.y - 1, value.z - 1, surface);
        DVXL xp1ym1zp1 = GetByIndexFromGridAndSurface(value.x + 0, value.y - 1, value.z + 0, surface);
        DVXL xp1yp1zm1 = GetByIndexFromGridAndSurface(value.x + 0, value.y + 0, value.z - 1, surface);
        DVXL xp1yp1zp1 = GetByIndexFromGridAndSurface(value.x + 0, value.y + 0, value.z + 0, surface);
        // Check the "z- down" edge
        if (object_id != xm1ym1zm1.object_id || object_id != xm1yp1zm1.object_id || object_id != xp1ym1zm1.object_id || object_id != xp1yp1zm1.object_id)
        {
            if (!(object_id != xm1ym1zm1.object_id && object_id != xm1yp1zm1.object_id && object_id != xp1ym1zm1.object_id && object_id != xp1yp1zm1.object_id))
            {
                edge_count++;
                vertex_connectivity[key] |= 0b00000001;
            }
        }
        // Check the "z+ up" edge
        if (object_id != xm1ym1zp1.object_id || object_id != xm1yp1zp1.object_id || object_id != xp1ym1zp1.object_id || object_id != xp1yp1zp1.object_id)
        {
            if (!(object_id != xm1ym1zp1.object_id && object_id != xm1yp1zp1.object_id && object_id != xp1ym1zp1.object_id && object_id != xp1yp1zp1.object_id))
            {
                edge_count++;
                vertex_connectivity[key] |= 0b00000010;
            }
        }
        // Check the "y- right" edge
        if (object_id != xm1ym1zm1.object_id || object_id != xm1ym1zp1.object_id || object_id != xp1ym1zm1.object_id || object_id != xp1ym1zp1.object_id)
        {
            if (!(object_id != xm1ym1zm1.object_id && object_id != xm1ym1zp1.object_id && object_id != xp1ym1zm1.object_id && object_id != xp1ym1zp1.object_id))
            {
                edge_count++;
                vertex_connectivity[key] |= 0b00000100;
            }
        }
        // Check the "y+ left" edge
        if (object_id != xm1yp1zm1.object_id || object_id != xm1yp1zp1.object_id || object_id != xp1yp1zm1.object_id || object_id != xp1yp1zp1.object_id)
        {
            if (!(object_id != xm1yp1zm1.object_id && object_id != xm1yp1zp1.object_id && object_id != xp1yp1zm1.object_id && object_id != xp1yp1zp1.object_id))
            {
                edge_count++;
                vertex_connectivity[key] |= 0b00001000;
            }
        }
        // Check the "x- back" edge
        if (object_id != xm1ym1zm1.object_id || object_id != xm1ym1zp1.object_id || object_id != xm1yp1zm1.object_id || object_id != xm1yp1zp1.object_id)
        {
            if (!(object_id != xm1ym1zm1.object_id && object_id != xm1ym1zp1.object_id && object_id != xm1yp1zm1.object_id && object_id != xm1yp1zp1.object_id))
            {
                edge_count++;
                vertex_connectivity[key] |= 0b00010000;
            }
        }
        // Check the "x+ front" edge
        if (object_id != xp1ym1zm1.object_id || object_id != xp1ym1zp1.object_id || object_id != xp1yp1zm1.object_id || object_id != xp1yp1zp1.object_id)
        {
            if (!(object_id != xp1ym1zm1.object_id && object_id != xp1ym1zp1.object_id && object_id != xp1yp1zm1.object_id && object_id != xp1yp1zp1.object_id))
            {
                edge_count++;
                vertex_connectivity[key] |= 0b00100000;
            }
        }
        // Increment M counts
        if (edge_count == 3)
        {
            M3++;
        }
        else if (edge_count == 5)
        {
            M5++;
        }
        else if (edge_count == 6)
        {
            M6++;
        }
    }
    // Check to see if the set of vertices is connected. If not, our object contains void(s)
    int32_t number_of_surfaces = ComputeConnectivityOfSurfaceVertices(vertex_connectivity);
    int32_t number_of_voids = number_of_surfaces - 1;
    // Compute the number of holes in the surface
    int32_t raw_number_of_holes = 1 + ((M5 + (2 * M6) - M3) / 8);
    int32_t number_of_holes = raw_number_of_holes + number_of_voids;
    if (verbose)
    {
        std::cout << "Processing surface with M3 = " << M3 << " M5 = " << M5 << " M6 = " << M6 << " holes = " << number_of_holes << " surfaces = " << number_of_surfaces << " voids = " << number_of_voids << std::endl;
    }
    return std::pair<int32_t, int32_t>(number_of_holes, number_of_voids);
}

int32_t DVXLGrid::ComputeConnectivityOfSurfaceVertices(const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& surface_vertices_connectivity) const
{
    int32_t connected_components = 0;
    int64_t processed_vertices = 0;
    size_t vertex_components_size_hint = surface_vertices_connectivity.size();
    std::unordered_map<VoxelGrid::GRID_INDEX, int32_t> vertex_components(vertex_components_size_hint);
    // Iterate through the vertices
    std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>::const_iterator surface_vertices_itr;
    for (surface_vertices_itr = surface_vertices_connectivity.begin(); surface_vertices_itr != surface_vertices_connectivity.end(); ++surface_vertices_itr)
    {
        VoxelGrid::GRID_INDEX key = surface_vertices_itr->first;
        VoxelGrid::GRID_INDEX location = key;
        //const uint8_t& connectivity = surface_vertices_itr->second.second;
        // First, check if the vertex has already been marked
        if (vertex_components[key] > 0)
        {
            continue;
        }
        else
        {
            // If not, we start marking a new connected component
            connected_components++;
            // Make the working queue
            std::list<VoxelGrid::GRID_INDEX> working_queue;
            // Make a hash table to store queued indices (so we don't repeat work)
            size_t queued_hashtable_size_hint = surface_vertices_connectivity.size();
            std::unordered_map<VoxelGrid::GRID_INDEX, int8_t> queued_hashtable(queued_hashtable_size_hint);
            // Add the current point
            working_queue.push_back(location);
            queued_hashtable[key] = 1;
            // Keep track of the number of vertices we've processed
            int64_t component_processed_vertices = 0;
            // Loop from the queue
            while (working_queue.size() > 0)
            {
                // Get the top of thw working queue
                VoxelGrid::GRID_INDEX current_vertex = working_queue.front();
                working_queue.pop_front();
                component_processed_vertices++;
                vertex_components[current_vertex] = connected_components;
                // Check the six possibly-connected vertices and add them to the queue if they are connected
                // Get the connectivity of our index
                std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>::const_iterator found_connectivity = surface_vertices_connectivity.find(current_vertex);
                if (found_connectivity == surface_vertices_connectivity.end())
                {
                    std::cerr << "Surface vertex list is incomplete" << std::endl;
                }
                uint8_t connectivity = found_connectivity->second;
                // Go through the neighbors
                if ((connectivity & 0b00000001) > 0)
                {
                    // Try to add the vertex
                    VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x, current_vertex.y, current_vertex.z - 1);
                    // We only add if we haven't already processed it
                    if (queued_hashtable[connected_vertex] <= 0)
                    {
                        queued_hashtable[connected_vertex] = 1;
                        working_queue.push_back(connected_vertex);
                    }
                }
                if ((connectivity & 0b00000010) > 0)
                {
                    // Try to add the vertex
                    VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x, current_vertex.y, current_vertex.z + 1);
                    // We only add if we haven't already processed it
                    if (queued_hashtable[connected_vertex] <= 0)
                    {
                        queued_hashtable[connected_vertex] = 1;
                        working_queue.push_back(connected_vertex);
                    }
                }
                if ((connectivity & 0b00000100) > 0)
                {
                    // Try to add the vertex
                    VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x, current_vertex.y - 1, current_vertex.z);
                    // We only add if we haven't already processed it
                    if (queued_hashtable[connected_vertex] <= 0)
                    {
                        queued_hashtable[connected_vertex] = 1;
                        working_queue.push_back(connected_vertex);
                    }
                }
                if ((connectivity & 0b00001000) > 0)
                {
                    // Try to add the vertex
                    VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x, current_vertex.y + 1, current_vertex.z);
                    // We only add if we haven't already processed it
                    if (queued_hashtable[connected_vertex] <= 0)
                    {
                        queued_hashtable[connected_vertex] = 1;
                        working_queue.push_back(connected_vertex);
                    }
                }
                if ((connectivity & 0b00010000) > 0)
                {
                    // Try to add the vertex
                    VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x - 1, current_vertex.y, current_vertex.z);
                    // We only add if we haven't already processed it
                    if (queued_hashtable[connected_vertex] <= 0)
                    {
                        queued_hashtable[connected_vertex] = 1;
                        working_queue.push_back(connected_vertex);
                    }
                }
                if ((connectivity & 0b00100000) > 0)
                {
                    // Try to add the vertex
                    VoxelGrid::GRID_INDEX connected_vertex(current_vertex.x + 1, current_vertex.y, current_vertex.z);
                    // We only add if we haven't already processed it
                    if (queued_hashtable[connected_vertex] <= 0)
                    {
                        queued_hashtable[connected_vertex] = 1;
                        working_queue.push_back(connected_vertex);
                    }
                }
            }
            processed_vertices += component_processed_vertices;
            if (processed_vertices == (int64_t)surface_vertices_connectivity.size())
            {
                break;
            }
        }
    }
    return connected_components;
}


