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

bool ContactExistenceConditionConstraint::insertInProblem()
{
    bool ok = TaskCommon::insertInProblem();
    ok &= wrench_.insertInProblem();
    return ok;
}

bool ContactExistenceConditionConstraint::removeFromProblem()
{
    bool ok = TaskCommon::removeFromProblem();
    ok &= wrench_.removeFromProblem();
    return ok;
}

ContactExistenceConditionConstraint::~ContactExistenceConditionConstraint()
{
    this->removeFromProblem();
}

void ContactExistenceConditionConstraint::setContactFrame(const std::string& contact_frame)
{
    wrench_.setControlFrame(contact_frame);
}

const std::string& ContactExistenceConditionConstraint::getContactFrame() const
{
    return wrench_.getControlFrame();
}

void ContactExistenceConditionConstraint::setBaseFrame(const std::string& base_ref_frame)
{
    wrench_.setBaseFrame(base_ref_frame);
}

const std::string& ContactExistenceConditionConstraint::getBaseFrame() const
{
    return wrench_.getBaseFrame();
}

const Wrench& ContactExistenceConditionConstraint::getWrench() const
{
    return wrench_;
}

void ContactExistenceConditionConstraint::updateConstraintFunction()
{
    MutexLock lock(mutex);

    wrench_.update();

    frame_vias_acc_ = - robot().getFrameBiasAcc(wrench_.getControlFrame());

    this->setConstraintMatrix( wrench_.getJacobian().topLeftCorner(3, wrench_.getJacobian().cols()) );
    this->setBound( frame_vias_acc_.head(3) );
}

const Eigen::MatrixXd& ContactExistenceConditionConstraint::getJacobianTranspose() const
{
    return wrench_.getJacobianTranspose();
}

void ContactExistenceConditionConstraint::resize()
{
    MutexLock lock(mutex);

    if(wrench_.robotPtr() != this->robotPtr() )
    {
        wrench_.setRobotModel( this->robotPtr() );
        constraintFunction().resize( 3 , wrench_.getJacobian().cols());
    }
}
