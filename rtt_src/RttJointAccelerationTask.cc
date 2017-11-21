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
#include <orca/rtt_orca/robot/RobotModelHelper.h>

namespace rtt_orca
{
namespace task
{
    class RttJointAccelerationTask: public orca::task::JointAccelerationTask, public RTT::TaskContext
    {
    public:
        RttJointAccelerationTask(const std::string& name)
        : RTT::TaskContext(name)
        , robotHelper_(this,this,this->robot())
        {
            orca::task::JointAccelerationTask::setName(name);
            this->addOperation("insertInProblem",&orca::task::GenericTask::insertInProblem,this,RTT::OwnThread);
            this->addOperation("removeFromProblem",&orca::task::GenericTask::removeFromProblem,this,RTT::OwnThread);
            
            this->provides("pid")->addOperation("setProportionalGain",&orca::common::PIDController<Eigen::Dynamic>::setProportionalGain,&this->pid(),RTT::OwnThread);
            this->provides("pid")->addOperation("setDerivativeGain",&orca::common::PIDController<Eigen::Dynamic>::setDerivativeGain,&this->pid(),RTT::OwnThread);
            
            this->addOperation("setDesired",&orca::task::JointAccelerationTask::setDesired,this,RTT::OwnThread);
        }

        bool configureHook()
        {
            robotHelper_.configureRobotPorts();
            return true;
        }

        void updateHook()
        {
            robotHelper_.updateRobotModel();
            port_jnt_pos_des_.readNewest(this->jnt_pos_des_);
            port_jnt_vel_des_.readNewest(this->jnt_vel_des_);
            port_jnt_acc_des_.readNewest(this->jnt_acc_des_);
            orca::task::JointAccelerationTask::update();
        }

    protected:
        robot::RobotModelHelper robotHelper_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_pos_des_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_vel_des_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_acc_des_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::task::RttJointAccelerationTask)
