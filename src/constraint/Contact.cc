#include <orca/constraint/Contact.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::constraint;
using namespace orca::optim;

Contact::Contact()
: TaskCommon(optim::ControlVariable::Composite)
, friction_cone_(new LinearizedCoulombConstraint)
, ex_condition_(new ContactExistenceConditionConstraint)
{
    this->desactivate();
}

void Contact::insertInProblem()
{
    friction_cone_->insertInProblem();
    ex_condition_->insertInProblem();
}

void Contact::removeFromProblem()
{
    friction_cone_->removeFromProblem();
    ex_condition_->removeFromProblem();
}

void Contact::desactivate()
{
    // TODO : salini p43 - Section 2.1.4.2 - When contact is desactivatied, add S*X = 0
    friction_cone_->desactivate();
    ex_condition_->desactivate();
}

void Contact::activate()
{
    friction_cone_->activate();
    ex_condition_->activate();
}

void Contact::update()
{
    friction_cone_->update();
    ex_condition_->update();
}

double Contact::getFrictionCoeff() const
{
    return friction_cone_->getFrictionCoeff();
}

double Contact::getMargin() const
{
    return friction_cone_->getMargin();
}

void Contact::setAngleOffset(double angle_offset)
{
    friction_cone_->setAngleOffset(angle_offset);
}

void Contact::setFrictionCoeff(double coeff)
{
    friction_cone_->setFrictionCoeff(coeff);
}

void Contact::setMargin(double margin)
{
    friction_cone_->setMargin(margin);
}

void Contact::setConeOrientation(const Eigen::Matrix3d& R)
{
    friction_cone_->setConeOrientation(R);
}

const Eigen::Matrix3d& Contact::getConeOrientation() const
{
    return friction_cone_->getConeOrientation();
}

void Contact::setNumberOfFaces(int nfaces)
{
    friction_cone_->setNumberOfFaces(nfaces);
}

const Eigen::MatrixXd& Contact::getJacobianTranspose() const
{
    return ex_condition_->getJacobianTranspose();
}

void Contact::setContactFrame(const std::string& contact_frame)
{
    ex_condition_->setContactFrame(contact_frame);
}

std::shared_ptr<LinearizedCoulombConstraint> Contact::getLinearizedCoulombConstraint() const
{
    return friction_cone_;
}

std::shared_ptr<ContactExistenceConditionConstraint> Contact::getContactExistenceConditionConstraint() const
{
    return ex_condition_;
}

void Contact::resize()
{
    if( ex_condition_->robotPtr() != this->robotPtr() )
        ex_condition_->setRobotModel( this->robotPtr() );
}
