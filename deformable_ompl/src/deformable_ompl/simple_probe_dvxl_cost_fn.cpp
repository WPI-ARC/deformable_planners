#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "deformable_ompl/dvxl_cost_fn.hpp"
#include "deformable_ompl/simple_probe_state_space.hpp"
#include "deformable_ompl/simple_probe_dvxl_cost_fn.hpp"

using namespace deformable_ompl;

bool CheckInLBlock(const double x, const double y)
{
    if (x <= 0.0 || y >= 0.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<std::pair<Eigen::Vector3d, DVXL>> SimpleProbeDVXLCostValidityChecker::MakeProbePoints(const double resolution) const
{
    std::vector<std::pair<Eigen::Vector3d, DVXL>> points;
    DVXL probe_dvxl;
    probe_dvxl.deformability = 0.0;
    probe_dvxl.sensitivity = 1.0;
    probe_dvxl.mass = 0.0;
    probe_dvxl.r = 0xff;
    probe_dvxl.g = 0x00;
    probe_dvxl.b = 0xff;
    probe_dvxl.a = 0xff;
    // Loop through all possible cells bounding the probe at 2.5 cm resolution
    int length_cells = (int)(0.04 * 2.0 * (1.0 / resolution));
    int width_cells = (int)(0.04 * 2.0 * (1.0 / resolution));
    for (int x_idx = 0; x_idx < length_cells; x_idx++)
    {
        for (int y_idx = 0; y_idx < width_cells; y_idx++)
        {
            double x_location = -(0.04 - (resolution * 0.5)) + (resolution * x_idx);
            double y_location = -(0.04 - (resolution * 0.5)) + (resolution * y_idx);
            double z_location = 0.0;
            if (CheckInLBlock(x_location, y_location))
            {
                Eigen::Vector3d point_location(x_location, y_location, z_location);
                std::pair<Eigen::Vector3d, DVXL> point(point_location, probe_dvxl);
                points.push_back(point);
            }
        }
    }
    return points;
}
