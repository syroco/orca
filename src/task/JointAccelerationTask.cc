#include "orca/task/JointAccelerationTask.h"
#include "orca/utils/Utils.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;
using namespace orca::utils;

JointAccelerationTask::JointAccelerationTask(const std::string& name)
: GenericTask(name,ControlVariable::JointAcceleration)
{
    this->addParameter("pid",&pid_);
    this->addParameter("jnt_pos_des",&jnt_pos_des_,ParamPolicy::Optional);
    this->addParameter("jnt_vel_des",&jnt_vel_des_,ParamPolicy::Optional);
    this->addParameter("jnt_acc_des",&jnt_acc_des_,ParamPolicy::Optional);
}

void JointAccelerationTask::setDesired(const Eigen::VectorXd& jnt_pos_des
                                    , const Eigen::VectorXd& jnt_vel_des
                                    , const Eigen::VectorXd& jnt_acc_des)
{
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
    // Set the desired position to the current robot state
    if(!jnt_pos_des_.isSet())
        jnt_pos_des_ = robot()->getJointPos();
        
    assertSize(jnt_pos_des_.get(),robot()->getNrOfDegreesOfFreedom());
    assertSize(jnt_vel_des_.get(),robot()->getNrOfDegreesOfFreedom());
    assertSize(jnt_acc_des_.get(),robot()->getNrOfDegreesOfFreedom());
}

void JointAccelerationTask::onUpdateAffineFunction(double current_time, double dt)
{
    const Eigen::VectorXd& current_jnt_pos = robot()->getJointPos();
    const Eigen::VectorXd& current_jnt_vel = robot()->getJointVel();

    f() = - (jnt_acc_des_.get() + pid_.get()->computeCommand( jnt_pos_des_.get() - current_jnt_pos , jnt_vel_des_.get() - current_jnt_vel , dt) );
}

void JointAccelerationTask::onResize()
{
    const unsigned int dof = robot()->getNrOfDegreesOfFreedom();

    if(this->cols() != dof)
    {
        euclidianNorm().resize(dof,dof);
        E().setIdentity();
        
        pid_.get()->resize(dof);
        
        if(!jnt_pos_des_.isSet())
            jnt_pos_des_ = Eigen::VectorXd::Zero( dof );
        if(!jnt_vel_des_.isSet())
            jnt_vel_des_ = Eigen::VectorXd::Zero( dof );
        if(!jnt_acc_des_.isSet())
            jnt_acc_des_ = Eigen::VectorXd::Zero( dof );
    }
}

ORCA_REGISTER_CLASS(orca::task::JointAccelerationTask)
