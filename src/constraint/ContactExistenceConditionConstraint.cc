#include <orca/constraint/ContactExistenceConditionConstraint.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::robot;
using namespace orca::common;

ContactExistenceConditionConstraint::ContactExistenceConditionConstraint()
: EqualityConstraint(ControlVariable::JointSpaceAcceleration)
{

}
void ContactExistenceConditionConstraint::setBaseFrame(const std::string& base_ref_frame)
{
    MutexLock lock(mutex);

    if(robot().frameExists(base_ref_frame))
        base_ref_frame_ = base_ref_frame;
    else
        LOG_ERROR << "Could not set base frame to " << base_ref_frame;
}

void ContactExistenceConditionConstraint::setControlFrame(const std::string& control_frame)
{
    MutexLock lock(mutex);

    if(robot().frameExists(control_frame))
        control_frame_ = control_frame;
    else
        LOG_ERROR << "Could not set control frame to " << control_frame;
}


void ContactExistenceConditionConstraint::updateConstraintFunction()
{
    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot().getBaseFrame();
    }

    frame_bias_acc_ = - robot().getFrameBiasAcc(control_frame_);

    if(base_ref_frame_ == robot().getBaseFrame())
    {
        jacobian_ = robot().getFrameFreeFloatingJacobian(control_frame_);
    }
    else
    {
        const unsigned int dof = robot().getNrOfDegreesOfFreedom();
        jacobian_.block(0,6,6,dof) = robot().getRelativeJacobian(base_ref_frame_,control_frame_);
    }

    this->setConstraintMatrix( jacobian_.topLeftCorner(3, jacobian_.cols()) );
    this->setBound( frame_bias_acc_.head(3) );
}

void ContactExistenceConditionConstraint::resize()
{
    MutexLock lock(mutex);

    const int fulldim = OptimisationVector().configurationSpaceDimension();

    if(constraintFunction().cols() != fulldim)
    {
        constraintFunction().resize( 3 , fulldim);
        jacobian_.setZero(6,fulldim);
    }
}
