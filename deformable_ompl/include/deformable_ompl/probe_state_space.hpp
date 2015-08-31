#include <ompl/base/StateSpace.h>
#include <Eigen/Geometry>
#include <map>
#include <unordered_map>
#include <arc_utilities/voxel_grid.hpp>
#include "deformable_ompl/PatchedSE3StateSpace.hpp"

#ifndef PROBE_STATE_SPACE_HPP
#define PROBE_STATE_SPACE_HPP

namespace deformable_ompl
{
    class ProbeStateSpace : public ompl::base::PatchedSE3StateSpace
    {
    public:

        class StateType : public ompl::base::PatchedSE3StateSpace::StateType
        {
        public:

            StateType(void) : ompl::base::PatchedSE3StateSpace::StateType() {}

            inline std::string getKey() const
            {
                Eigen::Quaterniond rot = getEigenRotation();
                std::string key = std::to_string(getX()) + "|" + std::to_string(getY()) + "|" + std::to_string(getZ()) + "|"  + std::to_string(rot.w()) + "|" + std::to_string(rot.x()) + "|" + std::to_string(rot.y()) + "|" + std::to_string(rot.z());
                return key;
            }

            inline Eigen::Vector3d getEigenPosition(void) const
            {
                return Eigen::Vector3d(getX(), getY(), getZ());
            }

            inline Eigen::Quaterniond getEigenRotation(void) const
            {
                const ompl::base::PatchedSO3StateSpace::StateType& so3_rotation = rotation();
                return Eigen::Quaterniond(so3_rotation.w, so3_rotation.x, so3_rotation.y, so3_rotation.z);
            }

            inline Eigen::Affine3d getEigenTransform(void) const
            {
                Eigen::Translation3d position(getEigenPosition());
                Eigen::Quaterniond rotation = getEigenRotation();
                Eigen::Affine3d transform = position * rotation;
                return transform;
            }

            inline void setEigenPosition(const Eigen::Vector3d new_position)
            {
                setX(new_position.x());
                setY(new_position.y());
                setZ(new_position.z());
            }

            inline void setEigenRotation(const Eigen::Quaterniond new_rotation)
            {
                ompl::base::PatchedSO3StateSpace::StateType& so3_rotation = rotation();
                so3_rotation.w = new_rotation.w();
                so3_rotation.x = new_rotation.x();
                so3_rotation.y = new_rotation.y();
                so3_rotation.z = new_rotation.z();
            }

            inline void setEigenTransform(const Eigen::Affine3d transform)
            {
                Eigen::Vector3d position = transform.translation();
                Eigen::Quaterniond rotation(transform.rotation());
                setEigenPosition(position);
                setEigenRotation(rotation);
            }
        };

        ProbeStateSpace(void) : ompl::base::PatchedSE3StateSpace()
        {
            setName("Probe_" + getName());
            type_ = 1000;
            lock();
        }

        void setR3Bounds(const ompl::base::RealVectorBounds &bounds)
        {
            setBounds(bounds);
        }

        const ompl::base::RealVectorBounds& getR3Bounds(void) const
        {
            return getBounds();
        }
    };
}

#endif // PROBE_STATE_SPACE_HPP

