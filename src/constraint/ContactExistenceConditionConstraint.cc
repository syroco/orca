#include "orca/constraint/ContactExistenceConditionConstraint.h"

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::robot;
using namespace orca::common;

ContactExistenceConditionConstraint::ContactExistenceConditionConstraint(const std::string& name)
: EqualityConstraint(name,ControlVariable::JointAcceleration)
{

}
void ContactExistenceConditionConstraint::setBaseFrame(const std::string& base_ref_frame)
{
    base_ref_frame_ = base_ref_frame;
}

void ContactExistenceConditionConstraint::setControlFrame(const std::string& control_frame)
{
    control_frame_ = control_frame;
}

void ContactExistenceConditionConstraint::onActivation()
{
    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot()->getBaseFrame();
    }
}

void ContactExistenceConditionConstraint::onUpdateConstraintFunction(double current_time, double dt)
{
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

void ContactExistenceConditionConstraint::onResize()
{
    const int fulldim = this->robot()->getConfigurationSpaceDimension();

    if(constraintFunction().cols() != fulldim)
    {
        constraintFunction().resize( 3 , fulldim);
        jacobian_.setZero(6,fulldim);
    }
}

ORCA_REGISTER_CLASS(orca::constraint::ContactExistenceConditionConstraint)
