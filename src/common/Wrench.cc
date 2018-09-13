#include "orca/common/Wrench.h"
#include "orca/utils/Utils.h"

using namespace orca::common;
using namespace orca::utils;
using namespace orca::robot;

Wrench::Wrench(const std::string& name)
: TaskBase(name,optim::ControlVariable::None)
{
    this->addParameter("base_frame",&base_ref_frame_,ParamPolicy::Optional);
    this->addParameter("control_frame",&control_frame_);
}

void Wrench::setCurrentValue(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor)
{
    current_wrench_ = current_wrench_from_ft_sensor;
}

const Eigen::Matrix<double,6,1>& Wrench::getCurrentValue() const
{
    return current_wrench_;
}

void Wrench::setBaseFrame(const std::string& base_ref_frame)
{
    base_ref_frame_ = base_ref_frame;
}

void Wrench::setControlFrame(const std::string& control_frame)
{
    control_frame_ = control_frame;
}

const std::string& Wrench::getBaseFrame() const
{
    return base_ref_frame_.get();
}

const std::string& Wrench::getControlFrame() const
{
    return control_frame_.get();
}

const Eigen::MatrixXd& Wrench::getJacobianTranspose() const
{
    return jacobian_transpose_;
}

const Eigen::MatrixXd& Wrench::getJacobian() const
{
    return jacobian_;
}

void Wrench::onActivation()
{
    if(base_ref_frame_.get().empty())
    {
        base_ref_frame_ = robot()->getBaseFrame();
    }
}

void Wrench::onDeactivated()
{
    current_wrench_.setZero();
}

void Wrench::onCompute(double current_time, double dt)
{
    if(getBaseFrame() == robot()->getBaseFrame())
    {
        jacobian_ = robot()->getFrameFreeFloatingJacobian(getControlFrame());
    }
    else
    {
        const unsigned int dof = robot()->getNrOfDegreesOfFreedom();
        jacobian_.block(0,6,6,dof) = robot()->getRelativeJacobian(getBaseFrame(),getControlFrame());
    }
    jacobian_transpose_ = jacobian_.transpose();
}

void Wrench::onResize()
{
    int fulldim = robot()->getConfigurationSpaceDimension();

    if(jacobian_transpose_.rows() != fulldim || jacobian_transpose_.cols() != 6)
    {
        current_wrench_.setZero();
        jacobian_transpose_.setZero(fulldim,6);
        jacobian_.setZero(6,fulldim);
    }
}

void Wrench::print() const
{
    std::cout << "[" << getName() << "]" << '\n';
    std::cout << " - Base frame : " << getBaseFrame() << '\n';
    std::cout << " - Control frame : " << getControlFrame() << '\n';
    std::cout << " - Current value : " << getCurrentValue().transpose() << '\n';
    std::cout << " - Current jacobian : \n" << getJacobian() << '\n';
}

ORCA_REGISTER_CLASS(orca::common::Wrench)
