#include <orca/task/JointAccelerationTask.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::task;
using namespace orca::optim;

JointAccelerationTask::JointAccelerationTask()
: GenericTask(ControlVariable::JointSpaceAcceleration)
{

}

void JointAccelerationTask::setDesired(const Eigen::VectorXd& jnt_pos_des
                                    , const Eigen::VectorXd& jnt_vel_des
                                    , const Eigen::VectorXd& jnt_acc_des)
{
    MutexLock lock(mutex);
    
    jnt_pos_des_ = jnt_pos_des;
    jnt_vel_des_ = jnt_vel_des;
    jnt_acc_des_ = jnt_acc_des;
}

void JointAccelerationTask::setProportionalGain(const Eigen::VectorXd& p_gain)
{
    MutexLock lock(mutex);
    
    P_gain_ = p_gain;
}

void JointAccelerationTask::setDerivativeGain(const Eigen::VectorXd& d_gain)
{
    MutexLock lock(mutex);
    
    D_gain_ = d_gain;
}

void JointAccelerationTask::updateAffineFunction()
{
    const Eigen::VectorXd& current_jnt_pos = robot().eigRobotState.jointPos;
    const Eigen::VectorXd& current_jnt_vel = robot().eigRobotState.jointVel;

    f() = ( - (jnt_acc_des_ + P_gain_.asDiagonal() * ( current_jnt_pos - jnt_pos_des_ ) + D_gain_.asDiagonal() * ( current_jnt_vel - jnt_vel_des_ ) ) );
}

void JointAccelerationTask::resize()
{
    MutexLock lock(mutex);
    
    const int fulldim = OptimisationVector().ConfigurationSpaceDimension();
    const int dof = robot().getNrOfDegreesOfFreedom();

    EuclidianNorm().resize(fulldim,fulldim);

    jnt_pos_des_.setZero(dof);
    jnt_vel_des_.setZero(dof);
    jnt_acc_des_.setZero(dof);

    P_gain_.setZero(dof);
    D_gain_.setZero(dof);

    E().setIdentity();
}
