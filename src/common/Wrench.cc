#include "orca/common/Wrench.h"
#include "orca/optim/ControlVariable.h"
#include "orca/utils/Utils.h"

using namespace orca::common;
using namespace orca::optim;
using namespace orca::utils;
using namespace orca::robot;

Wrench::Wrench(const std::string& name)
: TaskBase(name,ControlVariable::ExternalWrench)
{

}

Wrench::~Wrench()
{

}

void Wrench::setCurrentValue(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor)
{
    assertSize(current_wrench_from_ft_sensor,current_wrench_);
    current_wrench_ = current_wrench_from_ft_sensor;
}

const Eigen::Matrix<double,6,1>& Wrench::getCurrentValue() const
{
    return current_wrench_;
}


void Wrench::setBaseFrame(const std::string& base_ref_frame)
{
    if(robot()->frameExists(base_ref_frame))
        base_ref_frame_ = base_ref_frame;
    else
        LOG_ERROR << "[" << getName() << "] Could not set base frame to " << base_ref_frame;
}

void Wrench::setControlFrame(const std::string& control_frame)
{
    if(robot()->frameExists(control_frame))
        control_frame_ = control_frame;
    else
        LOG_ERROR << "[" << getName() << "] Could not set control frame to " << control_frame;
}

const std::string& Wrench::getBaseFrame() const
{
    return base_ref_frame_;
}

const std::string& Wrench::getControlFrame() const
{
    return control_frame_;
}

const Eigen::MatrixXd& Wrench::getJacobianTranspose() const
{
    if(isActivated())
        return jacobian_transpose_;
    return zero_;
}

const Eigen::MatrixXd& Wrench::getJacobian() const
{
    if(isActivated())
        return jacobian_;
    return zero_;
}

void Wrench::update(double current_time, double dt)
{
    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot()->getBaseFrame();
        LOG_WARNING << "[" << getName() << "] baseFrame is not set, setting it up to the robot base frame " << base_ref_frame_;
    }

    if(control_frame_.empty())
    {
        throw std::runtime_error(Formatter() << "[" << getName() << "] controlFrame is not set");
    }

    if(base_ref_frame_ == robot()->getBaseFrame())
    {
        jacobian_ = robot()->getFrameFreeFloatingJacobian(control_frame_);
    }
    else
    {
        const unsigned int dof = robot()->getNrOfDegreesOfFreedom();
        jacobian_.block(0,6,6,dof) = robot()->getRelativeJacobian(base_ref_frame_,control_frame_);
    }
    jacobian_transpose_ = jacobian_.transpose();
}

void Wrench::resize()
{
    int fulldim = robot()->getConfigurationSpaceDimension();

    if(jacobian_transpose_.rows() != fulldim || jacobian_transpose_.cols() != 6)
    {
        current_wrench_.setZero();
        jacobian_transpose_.setZero(fulldim,6);
        zero_.setZero(fulldim,6);
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
