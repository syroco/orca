#include "orca/constraint/JointVelocityLimitConstraint.h"
using namespace orca::constraint;
using namespace orca::optim;

JointVelocityLimitConstraint::JointVelocityLimitConstraint(const std::string& name)
: JointLimitConstraint(name,ControlVariable::JointSpaceAcceleration)
{}

void JointVelocityLimitConstraint::setHorizon(double horizon)
{
    horizon_ = horizon;
}

void JointVelocityLimitConstraint::onUpdateConstraintFunction(double current_time, double dt)
{
    const Eigen::VectorXd& current_jnt_vel = this->robot()->getJointVel();

    constraintFunction().lowerBound().noalias() = ( minLimit() - current_jnt_vel ) / horizon_ ;
    constraintFunction().upperBound().noalias() = ( maxLimit() - current_jnt_vel ) / horizon_ ;
}
