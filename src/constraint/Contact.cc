#include <orca/constraint/Contact.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::common;

Contact::Contact()
: TaskCommon(optim::ControlVariable::Composite)
, friction_cone_(std::make_shared<LinearizedCoulombConstraint>())
, ex_condition_(std::make_shared<ContactExistenceConditionConstraint>())
, wrench_(std::make_shared<Wrench>())
{
    this->desactivate();
}

void Contact::setName(const std::string& name)
{
    TaskCommon::setName(name);
    friction_cone_->setName(name + "_FrCone");
    ex_condition_->setName(name + "_ExCond");
    wrench_->setName(name + "_Wrench");
}

void Contact::addInRegister()
{
    
}

void Contact::removeFromRegister()
{
    
}

bool Contact::isInitialized() const
{
    return wrench_->isInitialized();
}
bool Contact::isActivated() const
{
    return wrench_->isActivated();
}
bool Contact::isInsertedInProblem() const
{
    return wrench_->isInsertedInProblem();
}

bool Contact::insertInProblem()
{
    return friction_cone_->insertInProblem()
        && ex_condition_->insertInProblem() 
        && wrench_->insertInProblem();
}

bool Contact::removeFromProblem()
{
    return friction_cone_->removeFromProblem() 
        && ex_condition_->removeFromProblem() 
        && wrench_->removeFromProblem();
}

bool Contact::desactivate()
{
    // TODO : salini p43 - Section 2.1.4.2 - When contact is desactivatied, add S*X = 0
    return friction_cone_->desactivate() 
        && ex_condition_->desactivate() 
        && wrench_->desactivate();
}

bool Contact::activate()
{
    return friction_cone_->activate() 
        && ex_condition_->activate() 
        && wrench_->activate();
}

void Contact::update()
{
    friction_cone_->update();
    ex_condition_->update();
    wrench_->update();
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

const std::string& Contact::getBaseFrame() const
{
    return wrench_->getBaseFrame();
}

const std::string& Contact::getControlFrame() const
{
    return wrench_->getControlFrame();
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

void Contact::setCurrentWrench(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor)
{
    wrench_->setCurrent(current_wrench_from_ft_sensor);
}

void Contact::resize()
{
    ex_condition_->setRobotModel( this->robotPtr() );
    friction_cone_->setRobotModel( this->robotPtr() );
    wrench_->setRobotModel( this->robotPtr() );
}
