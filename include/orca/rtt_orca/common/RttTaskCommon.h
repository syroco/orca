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
        , comm_(*comm)
        {
            comm->setName(name);
            
            owner->provides("robot_model")->addOperation("loadModelFromFile",&RttTaskCommon::loadRobotModel,this , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("print", &orca::robot::RobotDynTree::print, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("setBaseFrame", &orca::robot::RobotDynTree::setBaseFrame, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("setGravity", &orca::robot::RobotDynTree::setGravity, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addPort("input.JointPosition",port_jnt_pos_in_);
            owner->provides("robot_model")->addPort("input.JointVelocity",port_jnt_vel_in_);
            owner->provides("robot_model")->addPort("input.WorldToBase",port_world_to_base_in_);
            owner->provides("robot_model")->addPort("input.BaseVelocity",port_base_vel_in_);
            owner->provides("robot_model")->addPort("input.Gravity",port_gravity_in_);
            
            owner->addOperation("desactivate",&orca::common::TaskCommon::desactivate,comm,RTT::OwnThread);
            owner->addOperation("activate",&orca::common::TaskCommon::activate,comm,RTT::OwnThread);
            owner->addOperation("isActivated",&orca::common::TaskCommon::isActivated,comm,RTT::OwnThread);
            owner->addOperation("insertInProblem",&orca::common::TaskCommon::insertInProblem,comm,RTT::OwnThread);
            owner->addOperation("removeFromProblem",&orca::common::TaskCommon::removeFromProblem,comm,RTT::OwnThread);
            owner->addOperation("isInsertedInProblem",&orca::common::TaskCommon::isInsertedInProblem,comm,RTT::OwnThread);
            owner->addOperation("isInitialized",&orca::common::TaskCommon::isInitialized,comm,RTT::OwnThread);
            
            owner->addOperation("print",&orca::common::TaskCommon::print,comm,RTT::OwnThread);
        }
        
        bool updateRobotModel()
        {
            RTT::FlowStatus fspos = port_jnt_pos_in_.readNewest(robot_data_helper_.eigRobotState.jointPos);
            RTT::FlowStatus fsvel = port_jnt_vel_in_.readNewest(robot_data_helper_.eigRobotState.jointVel);
            RTT::FlowStatus fswtb = port_world_to_base_in_.readNewest(robot_data_helper_.eigRobotState.world_H_base);
            RTT::FlowStatus fsbve = port_base_vel_in_.readNewest(robot_data_helper_.eigRobotState.baseVel);
            RTT::FlowStatus fsgra = port_gravity_in_.readNewest(robot_data_helper_.eigRobotState.gravity);
            
            if(fspos == RTT::NoData || fsvel == RTT::NoData)
            {
                return false;
            }
            
            robot_.setRobotState(robot_data_helper_.eigRobotState.world_H_base
                        ,robot_data_helper_.eigRobotState.jointPos
                        ,robot_data_helper_.eigRobotState.baseVel
                        ,robot_data_helper_.eigRobotState.jointVel
                        ,robot_data_helper_.eigRobotState.gravity
                                                );
        }
        
        bool loadRobotModel(const std::string& file_url)
        {
            if(comm_.loadRobotModel(file_url))
                robot_data_helper_.resize(robot_.getRobotModel());
            else
                throw std::runtime_error("Could not load robot model");
            return true;
        }

    private:
        RTT::InputPort<Eigen::VectorXd> port_jnt_pos_in_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_vel_in_;
        RTT::InputPort<Eigen::Matrix4d> port_world_to_base_in_;
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_base_vel_in_;
        RTT::InputPort<Eigen::Vector3d> port_gravity_in_;
        orca::robot::RobotDynTree& robot_;
        orca::robot::RobotDataHelper robot_data_helper_;
        orca::common::TaskCommon& comm_;
    };
} // namespace common
} // namespace rtt_orca