#include "orca/common/Wrench.h"
#include "orca/utils/Utils.h"

using namespace orca::common;
using namespace orca::utils;
using namespace orca::robot;

Wrench::Wrench(const std::string& name)
: name_(name)
{}

void Wrench::setRobotModel(std::shared_ptr<RobotModel> robot)
{
    robot_ = robot;
    resize();
}

Wrench::~Wrench()
{}

const std::string& Wrench::getName() const
{
    return name_;
}

bool Wrench::isActivated() const
{
    return is_activated_;
}

void Wrench::activate()
{
    is_activated_ = true;
}

void Wrench::deactivate()
{
    is_activated_ = false;
}

void Wrench::setCurrentValue(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor)
{
    assertSize(current_wrench_from_ft_sensor,current_wrench_);
    current_wrench_ = current_wrench_from_ft_sensor;
}

const Eigen::Matrix<double,6,1>& Wrench::getCurrentValue() const
{
    if(is_activated_)
        return current_wrench_;
    return wrench_zero_;
}

void Wrench::setBaseFrame(const std::string& base_ref_frame)
{
    if(robot_->frameExists(base_ref_frame))
        base_ref_frame_ = base_ref_frame;
    else
        LOG_ERROR << "[" << getName() << "] Frame " << base_ref_frame << " does not exists";
}

void Wrench::setControlFrame(const std::string& control_frame)
{
    if(robot_->frameExists(control_frame))
        control_frame_ = control_frame;
    else
        LOG_ERROR << "[" << getName() << "] Frame " << control_frame << " does not exists";;
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
    if(is_activated_)
        return jacobian_transpose_;
    return jac_zero_;
}

const Eigen::MatrixXd& Wrench::getJacobian() const
{
    if(is_activated_)
        return jacobian_;
    return jac_zero_;
}

void Wrench::update(double current_time, double dt)
{
    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot_->getBaseFrame();
    }

    if(base_ref_frame_ == robot_->getBaseFrame())
    {
        jacobian_ = robot_->getFrameFreeFloatingJacobian(control_frame_);
    }
    else
    {
        const unsigned int dof = robot_->getNrOfDegreesOfFreedom();
        jacobian_.block(0,6,6,dof) = robot_->getRelativeJacobian(base_ref_frame_,control_frame_);
    }
    jacobian_transpose_ = jacobian_.transpose();
}

void Wrench::resize()
{
    int fulldim = robot_->getConfigurationSpaceDimension();

    if(jacobian_transpose_.rows() != fulldim || jacobian_transpose_.cols() != 6)
    {
        current_wrench_.setZero();
        wrench_zero_.setZero();
        jacobian_transpose_.setZero(fulldim,6);
        jac_zero_.setZero(fulldim,6);
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
