#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <time.h>
#include <chrono>
#include <xtf/xtf.hpp>

class MultivariateGaussian
{
protected:

    std::mt19937_64 prng_;
    double max_spread_;
    Eigen::VectorXd mean_;
    size_t size_;
    std::normal_distribution<double> gaussian_;

public:

    MultivariateGaussian(const std::mt19937_64& prng, double max_spread, const Eigen::VectorXd& mean) : prng_(prng), max_spread_(max_spread), mean_(mean), size_(mean_.rows()), gaussian_(0.0, 1.0)
    {
    }

    double GetSpread(size_t index)
    {
        double percent_placement = double(index) / double (size_);
        if (percent_placement <= 0.5)
        {
            return max_spread_ * percent_placement;
        }
        else
        {
            double effective_percent = 1.0 - percent_placement;
            return max_spread_ * effective_percent;
        }
    }

    Eigen::VectorXd Sample()
    {
        Eigen::VectorXd output;
        output.resize(size_, 1);
        for (size_t idx = 0; idx < size_; idx++)
        {
            double spread = GetSpread(idx);
            output(idx) = mean_(idx) + (gaussian_(prng_) * spread);
        }
        return output;
    }
};

std::vector<double> ConfineToJointLimits(std::vector<double>& raw)
{
    std::vector<double> safe(raw.size(), 0.0);
    if (raw[0] < 0.0)
    {
        safe[0] = 0.0;
    }
    else if (raw[0] > 64.0)
    {
        safe[0] = 64.0;
    }
    else
    {
        safe[0] = raw[0];
    }
    if (raw[1] < 0.0)
    {
        safe[1] = 0.0;
    }
    else if (raw[1] > 96.0)
    {
        safe[1] = 96.0;
    }
    else
    {
        safe[1] = raw[1];
    }
    return safe;
}

XTF::Trajectory SampleNewTrajectory(XTF::Trajectory& original, const std::mt19937_64& prng, double max_spread, double std_dev)
{
    // Make the noise generators
    std::vector<MultivariateGaussian> generators;
    Eigen::VectorXd means = Eigen::VectorXd::Zero(original.size());
    for (size_t idx = 0; idx < original.joint_names_.size(); idx++)
    {
        MultivariateGaussian generator(prng, max_spread, means);
        generators.push_back(generator);
    }
    // For each of the joints, sample noise
    std::vector<Eigen::VectorXd> noise(original.joint_names_.size());
    for (size_t idx = 0; idx < original.joint_names_.size(); idx++)
    {
        noise[idx] = generators[idx].Sample() * std_dev;
    }
    // Add the noise to the trajectory
    XTF::Trajectory sampled(original.uid_ + "_sampled", original.traj_type_, original.timing_, original.robot_, original.generator_ + "_sampled", original.joint_names_, original.tags_);
    for (size_t state_idx = 0; state_idx < original.size(); state_idx++)
    {
        XTF::State original_state = original[state_idx];
        // Skip the first state
        if (state_idx == 0)
        {
            sampled.push_back(original_state);
            continue;
        }
        // Skip the last state
        else if (state_idx == (original.size() - 1))
        {
            sampled.push_back(original_state);
            continue;
        }
        // Only add the noise to intermediate states
        else
        {
            for (size_t joint_idx = 0; joint_idx < original.joint_names_.size(); joint_idx++)
            {
                original_state.position_desired_[joint_idx] += noise[joint_idx](state_idx);
            }
            original_state.position_desired_ = ConfineToJointLimits(original_state.position_desired_);
            sampled.push_back(original_state);
        }
    }
    return sampled;
}

bool CheckTrajInJointLimits(XTF::Trajectory& trajectory)
{
    for (size_t idx = 0; idx < trajectory.size(); idx++)
    {
        XTF::State& current_state = trajectory[idx];
        double xt = current_state.position_desired_[0];
        double yt = current_state.position_desired_[1];
        if (xt < 0.0 || xt > 64.0)
        {
            return false;
        }
        else if (yt < 0.0 || yt > 96.0)
        {
            return false;
        }
        else
        {
            continue;
        }
    }
    return true;
}

int main(int argc, char** argv)
{
    if (argc >= 5)
    {
        std::string traj_file(argv[1]);
        int32_t num_samples = atoi(argv[2]);
        double max_spread = atof(argv[3]);
        double std_dev = atof(argv[4]);
        // Make the parser
        XTF::Parser parser;
        // Load the original trajectory
        XTF::Trajectory original = parser.ParseTraj(traj_file);
        // Generate samples
        int32_t samples = 0;
        while (samples < num_samples)
        {
            // Make the PRNG
            unsigned long seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::mt19937_64 prng(seed);
            XTF::Trajectory sample = SampleNewTrajectory(original, prng, max_spread, std_dev);
            // Check to make sure it's in joint limits
            bool traj_valid = CheckTrajInJointLimits(sample);
            if (traj_valid)
            {
                std::string sample_filename = "sample_" + std::to_string(samples) + ".xtf";
                parser.ExportTraj(sample, sample_filename);
                samples++;
                std::cout << "Generated sample " << samples << std::endl;
            }
            else
            {
                std::cerr << "Skipped sample due to joint limit violation" << std::endl;
            }
        }
        return 0;
    }
    else
    {
        std::cout << "Must be run with four arguments - xtf filename, number of samples, max spread for samples, and std dev for sampling" << std::endl;
        exit(1);
    }
}
