#include <orca/rtt_orca/task/RttGenericTask.h>

namespace rtt_orca
{
namespace task
{
    class RttJointAccelerationTask: public orca::task::JointAccelerationTask, public task::RttGenericTask
    {
    public:
        RttJointAccelerationTask(const std::string& name)
        : task::RttGenericTask(this,this,name)
        {
            this->addOperation("insertInProblem",&orca::task::GenericTask::insertInProblem,this,RTT::OwnThread);
            this->addOperation("removeFromProblem",&orca::task::GenericTask::removeFromProblem,this,RTT::OwnThread);
            
            this->provides("pid")->addOperation("setProportionalGain",&orca::common::PIDController<Eigen::Dynamic>::setProportionalGain,&this->pid(),RTT::OwnThread);
            this->provides("pid")->addOperation("setDerivativeGain",&orca::common::PIDController<Eigen::Dynamic>::setDerivativeGain,&this->pid(),RTT::OwnThread);
            
            this->addOperation("setDesired",&orca::task::JointAccelerationTask::setDesired,this,RTT::OwnThread);
        }

        void updateHook()
        {
            if(this->updateRobotModel())
            {
                port_jnt_pos_des_.readNewest(this->jnt_pos_des_);
                port_jnt_vel_des_.readNewest(this->jnt_vel_des_);
                port_jnt_acc_des_.readNewest(this->jnt_acc_des_);
                orca::task::JointAccelerationTask::update();
            }
        }

    protected:
        RTT::InputPort<Eigen::VectorXd> port_jnt_pos_des_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_vel_des_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_acc_des_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::task::RttJointAccelerationTask)
