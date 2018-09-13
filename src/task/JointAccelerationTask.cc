#include "orca/task/JointAccelerationTask.h"
#include "orca/utils/Utils.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;
using namespace orca::utils;

JointAccelerationTask::JointAccelerationTask(const std::string& name)
: GenericTask(name,ControlVariable::JointAcceleration)
, pid_(std::make_shared<PIDController>())
{
    this->addParameter("pid",&pid_);
}

void JointAccelerationTask::setDesired(const Eigen::VectorXd& jnt_pos_des
                                    , const Eigen::VectorXd& jnt_vel_des
                                    , const Eigen::VectorXd& jnt_acc_des)
{
    assertSize(jnt_pos_des,jnt_pos_des_);
    assertSize(jnt_vel_des,jnt_vel_des_);
    assertSize(jnt_acc_des,jnt_acc_des_);

    jnt_pos_des_ = jnt_pos_des;
    jnt_vel_des_ = jnt_vel_des;
    jnt_acc_des_ = jnt_acc_des;
}

PIDController::Ptr JointAccelerationTask::pid()
{
    return pid_.get();
}
void JointAccelerationTask::onActivation()
{
    jnt_pos_des_ = robot()->getJointPos();
}

void JointAccelerationTask::onUpdateAffineFunction(double current_time, double dt)
{
    const Eigen::VectorXd& current_jnt_pos = robot()->getJointPos();
    const Eigen::VectorXd& current_jnt_vel = robot()->getJointVel();

    f() = - (jnt_acc_des_ + pid_.get()->computeCommand( jnt_pos_des_ - current_jnt_pos , jnt_vel_des_ - current_jnt_vel , dt) );
}

void JointAccelerationTask::onResize()
{
    const unsigned int dof = robot()->getNrOfDegreesOfFreedom();

    if(this->cols() != dof)
    {
        euclidianNorm().resize(dof,dof);
        E().setIdentity();
        
        pid_.get()->resize(dof);
        
        jnt_pos_des_.setZero( dof );
        jnt_vel_des_.setZero( dof );
        jnt_acc_des_.setZero( dof );
    }
}

ORCA_REGISTER_CLASS(orca::task::JointAccelerationTask)
