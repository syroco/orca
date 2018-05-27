#include "orca/task/JointTorqueTask.h"
#include "orca/utils/Utils.h"


using namespace orca::task;
using namespace orca::optim;
using namespace orca::robot;
using namespace orca::common;
using namespace orca::utils;


JointTorqueTask::JointTorqueTask(const std::string& name)
: GenericTask(name,ControlVariable::JointSpaceTorque)
, pid_(std::make_shared<PIDController>())
{

}

void JointTorqueTask::setDesired(const Eigen::VectorXd& desired_joint_torque)
{
    assertSize(desired_joint_torque,current_jnt_trq_);
    jnt_trq_des_ = desired_joint_torque;
}

void JointTorqueTask::onActivation()
{
    jnt_trq_des_.setZero();
}

void JointTorqueTask::onUpdateAffineFunction(double current_time, double dt)
{
    f() = - pid_->computeCommand(current_jnt_trq_ - jnt_trq_des_ , dt);
}

void JointTorqueTask::setCurrent(const Eigen::VectorXd& current_joint_torque)
{
    assertSize(current_joint_torque,current_jnt_trq_);
    current_jnt_trq_ = current_joint_torque;
}

std::shared_ptr<PIDController> JointTorqueTask::pid()
{
    return pid_;
}

void JointTorqueTask::onResize()
{
    const unsigned int dof = robot()->getNrOfDegreesOfFreedom();

    if(this->cols() != dof)
    {
        euclidianNorm().resize(dof,dof);

        pid_->resize(dof);

        jnt_trq_des_.setZero(dof);
        current_jnt_trq_.setZero(dof);

        E().setIdentity();
    }
}
