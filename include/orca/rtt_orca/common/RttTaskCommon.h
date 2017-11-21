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
namespace common
{
    class RttTaskCommon: public RTT::TaskContext
    {
    public:
        RttTaskCommon(RTT::TaskContext* owner,orca::common::TaskCommon* comm,const std::string& name)
        : RTT::TaskContext(name)
        , robot_(comm->robot())
        {
            comm->setName(name);
            robot_data_helper_.resize(robot_.getRobotModel());
            owner->provides("robot_model")->addOperation("loadModelFromFile",&orca::common::TaskCommon::loadRobotModel, comm , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("print", &orca::robot::RobotDynTree::print, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("setBaseFrame", &orca::robot::RobotDynTree::setBaseFrame, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("setGravity", &orca::robot::RobotDynTree::setGravity, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addPort("input.JointPosition",port_jnt_pos_in_);
            owner->provides("robot_model")->addPort("input.JointVelocity",port_jnt_vel_in_);
            owner->provides("robot_model")->addPort("input.WorldToBase",port_world_to_base_in_);
            owner->provides("robot_model")->addPort("input.BaseVelocity",port_base_vel_in_);
            owner->provides("robot_model")->addPort("input.Gravity",port_gravity_in_);
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

    private:
        RTT::InputPort<Eigen::VectorXd> port_jnt_pos_in_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_vel_in_;
        RTT::InputPort<Eigen::Matrix4d> port_world_to_base_in_;
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_base_vel_in_;
        RTT::InputPort<Eigen::Vector3d> port_gravity_in_;
        orca::robot::RobotDynTree& robot_;
        orca::robot::RobotDataHelper robot_data_helper_;
    };
} // namespace common
} // namespace rtt_orca