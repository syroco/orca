#include "orca/constraint/Contact.h"

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::common;
using namespace orca::robot;

Contact::Contact(const std::string& name)
: TaskBase(name,ControlVariable::Composite)
, wrench_(std::make_shared<Wrench>(name + "_wrench"))
, friction_cone_(std::make_shared<LinearizedCoulombConstraint>(name + "_coulomb"))
, ex_condition_(std::make_shared<ContactExistenceConditionConstraint>(name + "_existence_condition"))
{
    this->desactivate();
}

bool Contact::desactivate()
{
    TaskBase::desactivate();
    // TODO : salini p43 - Section 2.1.4.2 - When contact is desactivatied, add S*X = 0
    return friction_cone_->desactivate()
        && ex_condition_->desactivate()
        && wrench_->desactivate();
}

bool Contact::activate()
{
    TaskBase::activate();
    return friction_cone_->activate()
        && ex_condition_->activate()
        && wrench_->activate();
}

const std::string& Contact::getBaseFrame() const
{
    return wrench_->getBaseFrame();
}

const std::string& Contact::getControlFrame() const
{
    return wrench_->getControlFrame();
}

void Contact::update(double current_time, double dt)
{
    wrench_->update(current_time,dt);
    friction_cone_->update(current_time,dt);
    ex_condition_->update(current_time,dt);
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
    wrench_->setBaseFrame(base_ref_frame);
}

void Contact::setControlFrame(const std::string& control_frame)
{
    ex_condition_->setControlFrame(control_frame);
    wrench_->setControlFrame(control_frame);
}

void Contact::setRobotModel(std::shared_ptr<RobotDynTree> robot)
{
    TaskBase::setRobotModel( robot );
    ex_condition_->setRobotModel( robot );
    friction_cone_->setRobotModel( robot );
    wrench_->setRobotModel( robot );
}
bool Contact::loadRobotModel(const std::string& file_url)
{
    TaskBase::loadRobotModel( file_url );
    ex_condition_->loadRobotModel( file_url );
    friction_cone_->loadRobotModel( file_url );
    return wrench_->loadRobotModel( file_url );
}

void Contact::setCurrentWrenchValue(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor)
{
    wrench_->setCurrentValue(current_wrench_from_ft_sensor);
}

std::shared_ptr<const Wrench> Contact::getWrench() const
{
    return wrench_;
}

void Contact::resize()
{
    ex_condition_->resize();
    friction_cone_->resize();
    wrench_->resize();
}
