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
#include "arc_utilities/eigen_helpers.hpp"
#include "deformable_ompl/simple_pid_controller.hpp"

#ifndef SIMPLE_UNCERTAINTY_MODELS_HPP
#define SIMPLE_UNCERTAINTY_MODELS_HPP

namespace simple_uncertainty_models
{
    class SimpleUncertainSensor
    {
    protected:

        bool initialized_;
        std::mt19937_64 rng_;
        std::uniform_real_distribution<double> noise_distribution_;

    public:

        SimpleUncertainSensor(std::mt19937_64& rng, const double noise_lower_bound, const double noise_upper_bound) : initialized_(true), rng_(rng), noise_distribution_(noise_lower_bound, noise_upper_bound) {}

        SimpleUncertainSensor() : initialized_(false), noise_distribution_(0.0, 0.0) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline double GetSensorValue(const double process_value)
        {
            double noise = noise_distribution_(rng_);
            return process_value + noise;
        }
    };

    class SimpleUncertainActuator
    {
    protected:

        bool initialized_;
        std::mt19937_64 rng_;
        std::uniform_real_distribution<double> noise_distribution_;

    public:

        SimpleUncertainActuator(std::mt19937_64& rng, const double noise_lower_bound, const double noise_upper_bound) : initialized_(true), rng_(rng), noise_distribution_(noise_lower_bound, noise_upper_bound) {}

        SimpleUncertainActuator() : initialized_(false), noise_distribution_(0.0, 0.0) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline double GetControlValue(const double control_input)
        {
            double noise = noise_distribution_(rng_);
            return control_input + noise;
        }
    };

    class Simple3dRobot
    {
    protected:

        bool initialized_;
        simple_pid_controller::SimplePIDController x_axis_controller_;
        simple_pid_controller::SimplePIDController y_axis_controller_;
        simple_pid_controller::SimplePIDController z_axis_controller_;
        simple_uncertainty_models::SimpleUncertainSensor x_axis_sensor_;
        simple_uncertainty_models::SimpleUncertainSensor y_axis_sensor_;
        simple_uncertainty_models::SimpleUncertainSensor z_axis_sensor_;
        simple_uncertainty_models::SimpleUncertainActuator x_axis_actuator_;
        simple_uncertainty_models::SimpleUncertainActuator y_axis_actuator_;
        simple_uncertainty_models::SimpleUncertainActuator z_axis_actuator_;
        Eigen::Vector3d position_;

    public:

        Simple3dRobot(std::mt19937_64& rng, const Eigen::Vector3d& initial_position, const double kp, const double ki, const double kd, const double integral_clamp, const double sensor_noise_mag, const double actuator_noise_mag)
        {
            x_axis_controller_ = simple_pid_controller::SimplePIDController(kp, ki, kd, integral_clamp);
            y_axis_controller_ = simple_pid_controller::SimplePIDController(kp, ki, kd, integral_clamp);
            z_axis_controller_ = simple_pid_controller::SimplePIDController(kp, ki, kd, integral_clamp);
            x_axis_sensor_ = simple_uncertainty_models::SimpleUncertainSensor(rng, -sensor_noise_mag, sensor_noise_mag);
            y_axis_sensor_ = simple_uncertainty_models::SimpleUncertainSensor(rng, -sensor_noise_mag, sensor_noise_mag);
            z_axis_sensor_ = simple_uncertainty_models::SimpleUncertainSensor(rng, -sensor_noise_mag, sensor_noise_mag);
            x_axis_actuator_ = simple_uncertainty_models::SimpleUncertainActuator(rng, -actuator_noise_mag, actuator_noise_mag);
            y_axis_actuator_ = simple_uncertainty_models::SimpleUncertainActuator(rng, -actuator_noise_mag, actuator_noise_mag);
            z_axis_actuator_ = simple_uncertainty_models::SimpleUncertainActuator(rng, -actuator_noise_mag, actuator_noise_mag);
            position_ = initial_position;
            initialized_ = true;
        }

        Simple3dRobot()
        {
            initialized_ = false;
        }

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline Eigen::Vector3d GetPosition() const
        {
            return position_;
        }

        inline void SetPosition(const Eigen::Vector3d& position)
        {
            position_ = position;
        }

        Eigen::Vector3d MoveTowardsTarget(const Eigen::Vector3d& target, const double step_size, std::function<Eigen::Vector3d(const Eigen::Vector3d&, const Eigen::Vector3d&)>& forward_simulation_fn)
        {
            Eigen::Vector3d start = GetPosition();
            double total_distance = (target - start).norm();
            int32_t num_steps = (int32_t)ceil(total_distance / step_size);
            num_steps = std::max(1, num_steps);
            for (int32_t step_num = 1; step_num <= num_steps; step_num++)
            {
                double percent = (double)step_num / (double)num_steps;
                Eigen::Vector3d current_target = EigenHelpers::Interpolate(start, target, percent);
                Eigen::Vector3d current = GetPosition();
                double x_axis_control = x_axis_actuator_.GetControlValue(x_axis_controller_.ComputeFeedbackTerm(current_target.x(), x_axis_sensor_.GetSensorValue(current.x()), 1.0));
                double y_axis_control = y_axis_actuator_.GetControlValue(y_axis_controller_.ComputeFeedbackTerm(current_target.y(), y_axis_sensor_.GetSensorValue(current.y()), 1.0));
                double z_axis_control = z_axis_actuator_.GetControlValue(z_axis_controller_.ComputeFeedbackTerm(current_target.z(), z_axis_sensor_.GetSensorValue(current.z()), 1.0));
                Eigen::Vector3d control(x_axis_control, y_axis_control, z_axis_control);
                Eigen::Vector3d result = forward_simulation_fn(current, control);
                SetPosition(result);
            }
            return GetPosition();
        }
    };

}

#endif // SIMPLE_UNCERTAINTY_MODELS_HPP
