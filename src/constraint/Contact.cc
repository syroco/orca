#include "orca/constraint/Contact.h"

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::common;
using namespace orca::robot;

Contact::Contact(const std::string& name)
: TaskBase(name,ControlVariable::ExternalWrench)
, friction_cone_(std::make_shared<LinearizedCoulombConstraint>(name + "_coulomb"))
, ex_condition_(std::make_shared<ContactExistenceConditionConstraint>(name + "_existence_condition"))
{
    this->link(friction_cone_);
    this->link(ex_condition_);
    //TODO : salini p43 - Section 2.1.4.2 - When contact is desactivatied, add S*X = 0;
}

const std::string& Contact::getBaseFrame() const
{
    return this->getWrench()->getBaseFrame();
}

const std::string& Contact::getControlFrame() const
{
    return this->getWrench()->getControlFrame();
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

void Contact::setBaseFrame(const std::string& base_ref_frame)
{
    ex_condition_->setBaseFrame(base_ref_frame);
    this->wrench()->setBaseFrame(base_ref_frame);
}

void Contact::setControlFrame(const std::string& control_frame)
{
    ex_condition_->setControlFrame(control_frame);
    this->wrench()->setControlFrame(control_frame);
}


void Contact::setCurrentWrenchValue(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor)
{
    this->wrench()->setCurrentValue(current_wrench_from_ft_sensor);
}


ORCA_REGISTER_CLASS(orca::constraint::Contact)
