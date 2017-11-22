#include <orca/task/JointTorqueTask.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::task;
using namespace orca::optim;
using namespace orca::robot;
using namespace orca::common;


JointTorqueTask::JointTorqueTask()
: GenericTask(ControlVariable::JointSpaceTorque)
, pid_(this->mutex)
{

}

void JointTorqueTask::setDesired(const Eigen::VectorXd& jnt_trq_des)
{
    MutexLock lock(mutex);
    
    jnt_trq_des_ = jnt_trq_des;
}

void JointTorqueTask::updateAffineFunction()
{
    f() = - pid_.computeCommand(current_jnt_trq_ - jnt_trq_des_);
}

void JointTorqueTask::setCurrent(const Eigen::VectorXd& jointTorque)
{
    MutexLock lock(mutex);
    
    current_jnt_trq_ = jointTorque;
}

PIDController<Eigen::Dynamic>& JointTorqueTask::pid()
{
    return pid_;
}

void JointTorqueTask::resize()
{
    MutexLock lock(mutex);
    
    const unsigned int dof = robot().getNrOfDegreesOfFreedom();

    if(this->cols() != dof)
    {
        EuclidianNorm().resize(dof,dof);

        pid_.resize(dof);

        jnt_trq_des_.setZero(dof);
        current_jnt_trq_.setZero(dof);

        E().setIdentity();
    }
}
