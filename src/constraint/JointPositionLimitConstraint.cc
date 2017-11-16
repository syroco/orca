#include <orca/constraint/JointPositionLimitConstraint.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::robot;

JointPositionLimitConstraint::JointPositionLimitConstraint()
: JointLimitConstraint(ControlVariable::JointSpaceAcceleration)
{}

void JointPositionLimitConstraint::setJointLimitsFromRobotModel()
{
    MutexLock lock(mutex);
    
    std::map<int, std::pair<double,double> > lim = robot().getJointPositionLimits();

    for(auto l : lim)
    {
        int i = l.first;
        double lb = l.second.first;
        double ub = l.second.second;
        this->min_[i] = lb;
        this->max_[i] = ub;
    }
}

void JointPositionLimitConstraint::setHorizon(double horizon)
{
    MutexLock lock(mutex);
    
    horizon_ = horizon;
}

void JointPositionLimitConstraint::update()
{
    MutexLock lock(mutex);
    
    const Eigen::VectorXd& min_jnt_pos(min_);
    const Eigen::VectorXd& max_jnt_pos(max_);

    const Eigen::VectorXd& current_jnt_pos = robot().eigRobotState.jointPos;
    const Eigen::VectorXd& current_jnt_vel = robot().eigRobotState.jointVel;

    constraintFunction().lowerBound().noalias() = 2. * ( min_jnt_pos - (current_jnt_pos + horizon_ * current_jnt_vel )) / ( horizon_ * horizon_ );
    constraintFunction().upperBound().noalias() = 2. * ( max_jnt_pos - (current_jnt_pos + horizon_ * current_jnt_vel )) / ( horizon_ * horizon_ );
}

void JointPositionLimitConstraint::resize()
{
    MutexLock lock(mutex);
    const int dof = robot().getNrOfDegreesOfFreedom();
    if(min_.size() != dof or max_.size() != dof)
    {
        JointLimitConstraint::resize();
        setJointLimitsFromRobotModel();
    }
}
