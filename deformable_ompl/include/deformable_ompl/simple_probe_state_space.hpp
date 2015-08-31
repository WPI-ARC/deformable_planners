#include <ompl/base/StateSpace.h>
#include <Eigen/Geometry>
#include <map>
#include <unordered_map>
#include <arc_utilities/voxel_grid.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#ifndef SIMPLE_PROBE_STATE_SPACE_HPP
#define SIMPLE_PROBE_STATE_SPACE_HPP

namespace deformable_ompl
{
    class SimpleProbeStateSpace : public ompl::base::RealVectorStateSpace
    {
    public:

        class StateType : public ompl::base::RealVectorStateSpace::StateType
        {
        public:

            StateType(void) : ompl::base::RealVectorStateSpace::StateType() {}

            inline std::string getKey(void) const
            {
                std::string key = std::to_string(getX()) + "|" + std::to_string(getY()) + "|" + std::to_string(getYaw());
                return key;
            }

            inline double getX(void) const
            {
                return values[0];
            }

            inline double getY(void) const
            {
                return values[1];
            }

            inline double getYaw(void) const
            {
                return values[2];
            }

            inline void setX(const double x)
            {
                values[0] = x;
            }

            inline void setY(const double y)
            {
                values[1] = y;
            }

            inline void setYaw(const double yaw)
            {
                values[2] = yaw;
            }

            inline Eigen::Vector3d getEigenPosition(void) const
            {
                return Eigen::Vector3d(getX(), getY(), 0.0);
            }

            inline double getRotation(void) const
            {
                return getYaw();
            }

            inline Eigen::Quaterniond getEigenRotation(void) const
            {
                double yaw = getYaw();
                return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            }

            inline Eigen::Affine3d getEigenTransform(void) const
            {
                Eigen::Translation3d position(getEigenPosition());
                Eigen::Quaterniond rotation = getEigenRotation();
                Eigen::Affine3d transform = position * rotation;
                return transform;
            }

            inline void setPosition(const double x, const double y)
            {
                setX(x);
                setY(y);
            }

            inline void setRotation(const double rotation)
            {
                setYaw(rotation);
            }

            inline void setPositionAndRotation(const double x, const double y, const double rotation)
            {
                setPosition(x, y);
                setRotation(rotation);
            }
        };

        SimpleProbeStateSpace(void) : ompl::base::RealVectorStateSpace(3)
        {
            setName("Simple_probe_" + getName());
            type_ = 1001;
        }

        virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const
        {
            double s1x = state1->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getX();
            double s1y = state1->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getY();
            double s1yaw = state1->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getYaw();
            double s2x = state2->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getX();
            double s2y = state2->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getY();
            double s2yaw = state2->as<deformable_ompl::SimpleProbeStateSpace::StateType>()->getYaw();
            double pos_dist = sqrt(pow((s1x - s2x), 2.0) + pow((s1y - s2y), 2.0));
            double rot_dist = 0.05 * fabs(s1yaw - s2yaw);
            return pos_dist + rot_dist;
        }
    };
}

#endif // SIMPLE_PROBE_STATE_SPACE_HPP

