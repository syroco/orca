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

void ContactExistenceConditionConstraint::insertInProblem()
{
    if(!registered_)
    {
        OptimisationVector().addInRegister(this);
        wrench_.insertInProblem();
        registered_ = true;
    }
}

void ContactExistenceConditionConstraint::removeFromProblem()
{
    if(registered_)
    {
        OptimisationVector().removeFromRegister(this);
        wrench_.removeFromProblem();
        registered_ = false;
    }
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

void ContactExistenceConditionConstraint::update()
{
    wrench_.update();

    frame_vias_acc_ = - iDynTree::toEigen(wrench_.robot().kinDynComp.getFrameBiasAcc(wrench_.getControlFrame()));

    this->setConstraintMatrix( wrench_.getJacobian().topLeftCorner(3, wrench_.getJacobian().cols()) );
    this->bound() = ( frame_vias_acc_.head(3) );
    this->setLowerBound( this->getBound() );
}

const Eigen::MatrixXd& ContactExistenceConditionConstraint::getJacobianTranspose() const
{
    return wrench_.getJacobianTranspose();
}

void ContactExistenceConditionConstraint::resize()
{
    if(wrench_.robotPtr() != this->robotPtr() )
        wrench_.setRobotModel( this->robotPtr() );
}
