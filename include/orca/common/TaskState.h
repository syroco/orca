// This file is a part of the ORCA framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// ORCA is a whole-body reactive controller framework for robotics.
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/

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
        ControlVariable control_variable;

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
            double weight;
        }
        State state;

        struct Maths
        {
            int rows;
            int cols;

            Eigen::MatrixXd selection_matrix;
            Eigen::MatrixXd Weight;

            Eigen::MatrixXd E;
            Eigen::VectorXd f;
            Eigen::MatrixXd Hessian;
            Eigen::MatrixXd Gradient;

            Eigen::VectorXd lower_bound;
            Eigen::VectorXd upper_bound;
            Eigen::MatrixXd constraint_matrix;
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
