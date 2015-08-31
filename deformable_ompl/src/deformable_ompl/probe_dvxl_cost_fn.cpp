#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "deformable_ompl/dvxl_cost_fn.hpp"
#include "deformable_ompl/probe_state_space.hpp"
#include "deformable_ompl/probe_dvxl_cost_fn.hpp"

using namespace deformable_ompl;

bool CheckInCircle(const double cx, const double cy, const double x, const double y, const double r)
{
    double dx = fabs(x - cx);
    double dy = fabs(y - cy);
    double dist = sqrt((dx * dx) + (dy * dy));
    if (dist <= fabs(r))
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<std::pair<Eigen::Vector3d, DVXL>> ProbeDVXLCostValidityChecker::MakeProbePoints(const double probe_radius, const double resolution) const
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
    int diameter_cells = (int)(probe_radius * 2.0 * (1.0 / resolution));
    int length_cells = (int)(PROBE_LENGTH * (1.0 / resolution));
    for (int x_idx = 0; x_idx < diameter_cells; x_idx++)
    {
        for (int y_idx = 0; y_idx < diameter_cells; y_idx++)
        {
            for (int z_idx = 0; z_idx < length_cells; z_idx++)
            {
                double x_location = -(probe_radius - (resolution * 0.5)) + (resolution * x_idx);
                double y_location = -(probe_radius - (resolution * 0.5)) + (resolution * y_idx);
                double z_location = -((PROBE_LENGTH * 0.5) - (resolution * 0.5)) + (resolution * z_idx);
                if (CheckInCircle(0.0, 0.0, x_location, y_location, probe_radius))
                {
                    Eigen::Vector3d point_location(x_location, y_location, z_location);
                    std::pair<Eigen::Vector3d, DVXL> point(point_location, probe_dvxl);
                    points.push_back(point);
                }
            }
        }
    }
    return points;
}
