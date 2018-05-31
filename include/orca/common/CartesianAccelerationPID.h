// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
//
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
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

#pragma once
#include "orca/math/Utils.h"
#include "orca/common/CartesianServoController.h"
#include "orca/common/PIDController.h"
#include "orca/common/TaskBase.h"

namespace orca
{
    namespace common
    {
        using math::Vector6d;

        class CartesianAccelerationPID : public CartesianServoController
        {
        public:
            /**
            * @brief The cartesian acceleration PID helps any cartesian task servo around a desired
            * cartesian pose, velocity and acceleration.
            * \f[
            *   \begin{split}
            *    CartPID & = \ddot{X}_{des} + PID(X,\dot{X}) \\
            *            & = \ddot{X}_{des} + K_P (X_{curr} - X_{des}) + K_I (X_{curr} - X_{des}) +  K_D (\dot{X}_{curr} - \dot{X}_{des})
            *    \end{split}
            * \f]
            * 
            * @param name The name of the component
            */
            CartesianAccelerationPID(const std::string& name);

            /**
            * @brief Set the control frame desired cartesian pose w.r.t the base frame.
            * This is usually something to get from a trajectory generator
            * 
            * @param cartesian_position_traj The desired pose w.r.t the base frame
            * @param cartesian_velocity_traj The desired cartesian velocity in mixed representation
            * @param cartesian_acceleration_traj The desired cartesian acceleration in mixed representation
            */
            void setDesired(const Eigen::Matrix4d& cartesian_position_traj
                        , const Vector6d& cartesian_velocity_traj
                        , const Vector6d& cartesian_acceleration_traj);
            /**
            * @brief Returns the computed command
            * 
            * @return const orca::math::Vector6d&
            */
            const Vector6d& getCommand() const;
            /**
            * @brief Returns the current pose of the control frame (w.r.t the base frame)
            * 
            * @return const Eigen::Matrix4d&
            */
            const Eigen::Matrix4d& getCurrentCartesianPose() const;
            /**
            * @brief Returns the current cartesian velocity in mixed representation
            * 
            * @return const orca::math::Vector6d&
            */
            const Vector6d& getCurrentCartesianVelocity() const;
            /**
            * @brief Returns desired pose (set with #setDesired)
            * 
            * @return const Eigen::Matrix4d&
            */
            const Eigen::Matrix4d& getDesiredCartesianPose() const;
            /**
            * @brief Returns the desired cartesian acceleration (mixed representation)
            * 
            * @return const orca::math::Vector6d&
            */
            const Vector6d& getDesiredCartesianVelocity() const;
            /**
            * @brief Return the desired cartesian acceleration
            * 
            * @return const orca::math::Vector6d&
            */
            const Vector6d& getDesiredCartesianAcceleration() const;
            /**
            * @brief Returns the computed pose error as a 6D Vector [dx,dy,dz,drx,dry,drz]
            * 
            * @return const orca::math::Vector6d&
            */
            const Vector6d& getCartesianPoseError() const;
            /**
            * @brief Returns the computed cartesian velocity
            * 
            * @return const orca::math::Vector6d&
            */
            const Vector6d& getCartesianVelocityError() const;
            /**
            * @brief Outputs info on std::cout
            * 
            */
            void print() const;
            /**
            * @brief The dimension 6 pid controller
            * 
            * @return std::shared_ptr< orca::common::PIDController >
            */
            std::shared_ptr<PIDController> pid();
        protected:
            void onResize();
            void onActivation();
            void onUpdate(double current_time, double dt);
            void onDeactivation(){}
        private:
            Eigen::Matrix4d cart_pos_curr_,cart_pos_des_;
            Vector6d cart_acc_cmd_
                    ,cart_acc_bias_
                    ,cart_acc_des_
                    ,cart_vel_des_
                    ,cart_pos_err_
                    ,cart_vel_err_
                    ,cart_vel_curr_;
            std::shared_ptr<PIDController> pid_;
            bool desired_set_ = false;
        };
    }
}
