#include <orca/task/JointTorqueTask.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::task;
using namespace orca::optim;
using namespace orca::robot;


JointTorqueTask::JointTorqueTask()
: GenericTask(ControlVariable::JointSpaceTorque)
{

}

void JointTorqueTask::setDesired(const Eigen::VectorXd& jnt_trq_des)
{
    MutexLock lock(mutex);
    
    jnt_trq_des_ = jnt_trq_des;
}

void JointTorqueTask::setProportionalGain(const Eigen::VectorXd& p_gain)
{
    MutexLock lock(mutex);
    
    P_gain_ = p_gain;
}

void JointTorqueTask::setDerivativeGain(const Eigen::VectorXd& d_gain)
{
    MutexLock lock(mutex);
    
    D_gain_ = d_gain;
}

void JointTorqueTask::updateAffineFunction()
{
    f() = - ( P_gain_.asDiagonal() * ( current_jnt_trq_ - jnt_trq_des_ ) );
}

void JointTorqueTask::setRobotState(const Eigen::VectorXd& jointTorque)
{
    MutexLock lock(mutex);
    
    current_jnt_trq_ = jointTorque;
}

void JointTorqueTask::resize()
{
    MutexLock lock(mutex);
    
    const int dof = robot().getNrOfDegreesOfFreedom();

    EuclidianNorm().resize(dof,dof);

    jnt_trq_des_.setZero(dof);
    current_jnt_trq_.setZero(dof);

    P_gain_.setZero(dof);
    D_gain_.setZero(dof);

    E().setIdentity();
}
