#include "orca/constraint/JointPositionLimitConstraint.h"


using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::robot;
using namespace orca::common;

JointPositionLimitConstraint::JointPositionLimitConstraint(const std::string& name)
: JointLimitConstraint(name,ControlVariable::JointSpaceAcceleration)
{}

void JointPositionLimitConstraint::setJointLimitsFromRobotModel()
{
    this->minLimit() = robot()->getMinJointPos();
    this->maxLimit() = robot()->getMaxJointPos();
}

void JointPositionLimitConstraint::setHorizon(double horizon)
{
    horizon_ = horizon;
}

void JointPositionLimitConstraint::onActivation()
{
    setJointLimitsFromRobotModel();
}

void JointPositionLimitConstraint::onUpdateConstraintFunction(double current_time, double dt)
{
    const Eigen::VectorXd& current_jnt_pos = robot()->getJointPos();
    const Eigen::VectorXd& current_jnt_vel = robot()->getJointVel();

    constraintFunction().lowerBound().noalias() = 2. * ( minLimit() - (current_jnt_pos + horizon_ * current_jnt_vel )) / ( horizon_ * horizon_ );
    constraintFunction().upperBound().noalias() = 2. * ( maxLimit() - (current_jnt_pos + horizon_ * current_jnt_vel )) / ( horizon_ * horizon_ );
}
