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
    for(auto l : robot()->getJointPositionLimits())
    {
        int i = l.first;
        double lb = l.second.first;
        double ub = l.second.second;
        this->minLimit()[i] = lb;
        this->maxLimit()[i] = ub;
    }
}

void JointPositionLimitConstraint::setHorizon(double horizon)
{
    horizon_ = horizon;
}

void JointPositionLimitConstraint::updateConstraintFunction(double current_time, double dt)
{
    const Eigen::VectorXd& current_jnt_pos = robot()->getJointPos();
    const Eigen::VectorXd& current_jnt_vel = robot()->getJointVel();

    constraintFunction().lowerBound().noalias() = 2. * ( minLimit() - (current_jnt_pos + horizon_ * current_jnt_vel )) / ( horizon_ * horizon_ );
    constraintFunction().upperBound().noalias() = 2. * ( maxLimit() - (current_jnt_pos + horizon_ * current_jnt_vel )) / ( horizon_ * horizon_ );
}

void JointPositionLimitConstraint::resize()
{
    const unsigned int dof = robot()->getNrOfDegreesOfFreedom();
    if(minLimit().size() != dof || maxLimit().size() != dof)
    {
        JointLimitConstraint::resize();
        setJointLimitsFromRobotModel();
    }
}
