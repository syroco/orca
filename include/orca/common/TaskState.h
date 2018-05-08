#pragma once

#include <Eigen/Core>
#include "orca/optim/ControlVariable.h"

namespace orca
{
namespace common
{
    struct TaskState
    {
        std::string name;

        struct Frames
        {
            std::string base_frame;
            std::string control_frame;
        }
        Frames frames;

        struct State
        {
            bool is_initialized;
            bool is_activated;
            double update_time;
            double dt;
        }
        State state;

        struct Maths
        {
            int rows;
            int cols;
            Eigen::MatrixXd E;
            Eigen::VectorXd f;
            Eigen::MatrixXd Hessian;
            Eigen::MatrixXd Gradient;
        }
        Maths maths;

        struct Current
        {
            Eigen::VectorXd joint_position;
            Eigen::VectorXd joint_velocity;

            Eigen::VectorXd control_frame_position;
            Eigen::VectorXd control_frame_velocity;

            Eigen::VectorXd wrench_value;
            Eigen::MatrixXd wrench_jacobian;
        }
        Current current;

        struct Error
        {
            Eigen::VectorXd joint_error;

            Eigen::VectorXd control_frame_error;

            Eigen::VectorXd wrench_error;
        }
        Error error;

        struct PID
        {
            Eigen::VectorXd P;
            Eigen::VectorXd I;
            Eigen::VectorXd windup_limit;
            Eigen::VectorXd D;

        }
        PID pid;
    };
} // namespace common
} // namespace orca
