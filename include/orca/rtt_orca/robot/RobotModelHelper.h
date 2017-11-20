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

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Operation.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Property.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include <orca/orca.h>

namespace rtt_orca
{
    struct RobotModelHelper
    {
        RobotModelHelper(RTT::TaskContext *owner,orca::common::TaskCommon& comp,orca::robot::RobotDynTree& robot)
        : robot_(robot)
        {
            robot_data_helper_.resize(robot.getRobotModel());
            owner->provides("robot_model")->addOperation("loadModelFromFile",&orca::common::TaskCommon::loadRobotModel, &comp , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("print", &orca::robot::RobotDynTree::print, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("setBaseFrame", &orca::robot::RobotDynTree::setBaseFrame, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addPort("JointPosition-in",port_jnt_pos_in_);
            owner->provides("robot_model")->addPort("JointVelocity-in",port_jnt_vel_in_);
            owner->provides("robot_model")->addPort("WorldToBase-in",port_world_to_base_in_);
            owner->provides("robot_model")->addPort("BaseVelocity-in",port_base_vel_in_);
            owner->provides("robot_model")->addPort("Gravity-in",port_gravity_in_);
        }

        void configureRobotPorts()
        {
            port_jnt_pos_in_.setDataSample(robot_data_helper_.eigRobotState.jointPos);
            port_jnt_vel_in_.setDataSample(robot_data_helper_.eigRobotState.jointVel);
            port_world_to_base_in_.setDataSample(robot_data_helper_.eigRobotState.world_H_base);
            port_base_vel_in_.setDataSample(robot_data_helper_.eigRobotState.baseVel);
        }

        void updateRobotModel()
        {
            port_jnt_pos_in_.readNewest(robot_data_helper_.eigRobotState.jointPos);
            port_jnt_vel_in_.readNewest(robot_data_helper_.eigRobotState.jointVel);
            port_world_to_base_in_.readNewest(robot_data_helper_.eigRobotState.world_H_base);
            port_base_vel_in_.readNewest(robot_data_helper_.eigRobotState.baseVel);
            port_gravity_in_.readNewest(robot_data_helper_.eigRobotState.gravity);

            robot_.setRobotState(robot_data_helper_.eigRobotState.world_H_base
                        ,robot_data_helper_.eigRobotState.jointPos
                        ,robot_data_helper_.eigRobotState.baseVel
                        ,robot_data_helper_.eigRobotState.jointVel
                        ,robot_data_helper_.eigRobotState.gravity
                                                );
        }

        RTT::InputPort<Eigen::VectorXd> port_jnt_pos_in_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_vel_in_;
        RTT::InputPort<Eigen::Matrix4d> port_world_to_base_in_;
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_base_vel_in_;
        RTT::InputPort<Eigen::Vector3d> port_gravity_in_;
        orca::robot::RobotDynTree& robot_;
        orca::robot::RobotDataHelper robot_data_helper_;
    };

}


