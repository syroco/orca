#include "orca/constraint/ContactExistenceConditionConstraint.h"

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::robot;
using namespace orca::common;

ContactExistenceConditionConstraint::ContactExistenceConditionConstraint(const std::string& name)
: EqualityConstraint(name,ControlVariable::JointSpaceAcceleration)
{

}
void ContactExistenceConditionConstraint::setBaseFrame(const std::string& base_ref_frame)
{
    if(robot()->frameExists(base_ref_frame))
        base_ref_frame_ = base_ref_frame;
    else
        LOG_ERROR << "Could not set base frame to " << base_ref_frame;
}

void ContactExistenceConditionConstraint::setControlFrame(const std::string& control_frame)
{
    if(robot()->frameExists(control_frame))
        control_frame_ = control_frame;
    else
        LOG_ERROR << "Could not set control frame to " << control_frame;
}


void ContactExistenceConditionConstraint::updateConstraintFunction(double current_time, double dt)
{
    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot()->getBaseFrame();
    }

    frame_bias_acc_ = - robot()->getFrameBiasAcc(control_frame_);

    if(base_ref_frame_ == robot()->getBaseFrame())
    {
        jacobian_ = robot()->getFrameFreeFloatingJacobian(control_frame_);
    }
    else
    {
        const unsigned int dof = robot()->getNrOfDegreesOfFreedom();
        jacobian_.block(0,6,6,dof) = robot()->getRelativeJacobian(base_ref_frame_,control_frame_);
    }

    this->setConstraintMatrix( jacobian_.topLeftCorner(3, jacobian_.cols()) );
    this->setBound( frame_bias_acc_.head(3) );
}

void ContactExistenceConditionConstraint::resize()
{
    const int fulldim = this->robot()->getConfigurationSpaceDimension();

    if(constraintFunction().cols() != fulldim)
    {
        constraintFunction().resize( 3 , fulldim);
        jacobian_.setZero(6,fulldim);
    }
}
