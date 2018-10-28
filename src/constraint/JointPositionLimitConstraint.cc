#include "orca/constraint/JointPositionLimitConstraint.h"

namespace orca
{
namespace constraint
{
using namespace orca::optim;
using namespace orca::robot;
using namespace orca::common;

JointPositionLimitConstraint::JointPositionLimitConstraint(const std::string& name)
: JointLimitConstraint(name,ControlVariable::JointAcceleration)
{
    this->addParameter("horizon",&horizon_);
}

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

    double horizon_dt = horizon_.get() * dt;

    constraintFunction().lowerBound().noalias() = 2. * ( minLimit() - (current_jnt_pos + horizon_dt * current_jnt_vel )) / ( horizon_dt * horizon_dt );
    constraintFunction().upperBound().noalias() = 2. * ( maxLimit() - (current_jnt_pos + horizon_dt * current_jnt_vel )) / ( horizon_dt * horizon_dt );
}

} // namespace constraint
} // namespace orca

ORCA_REGISTER_CLASS(orca::constraint::JointPositionLimitConstraint)
