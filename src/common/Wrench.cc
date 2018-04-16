#include "orca/common/Wrench.h"
#include "orca/optim/ControlVariable.h"

using namespace orca::common;
using namespace orca::optim;

Wrench::Wrench()
: TaskBase(ControlVariable::ExternalWrench)
{
    
}


void Wrench::setCurrent(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor)
{
    // MutexLock lock(this->mutex);

    current_wrench_ = current_wrench_from_ft_sensor;
}

const Eigen::Matrix<double,6,1>& Wrench::getCurrent()
{
    // MutexLock lock(this->mutex);

    return current_wrench_;
}


void Wrench::setBaseFrame(const std::string& base_ref_frame)
{
    // MutexLock lock(this->mutex);

    if(this->robot()->frameExists(base_ref_frame))
        base_ref_frame_ = base_ref_frame;
    else
        LOG_ERROR << "Could not set base frame to " << base_ref_frame;
}

void Wrench::setControlFrame(const std::string& control_frame)
{
    // MutexLock lock(this->mutex);

    if(this->robot()->frameExists(control_frame))
        control_frame_ = control_frame;
    else
        LOG_ERROR << "Could not set control frame to " << control_frame;
}

const std::string& Wrench::getBaseFrame() const
{
    // MutexLock lock(this->mutex);

    return base_ref_frame_;
}

const std::string& Wrench::getControlFrame() const
{
    // MutexLock lock(this->mutex);

    return control_frame_;
}

const Eigen::MatrixXd& Wrench::getJacobianTranspose() const
{
    // MutexLock lock(this->mutex);

    return jacobian_transpose_;
}

const Eigen::MatrixXd& Wrench::getJacobian() const
{
    // MutexLock lock(this->mutex);

    return jacobian_;
}

void Wrench::update()
{
    // MutexLock lock(this->mutex);

    // A task is considered initialised when
    // Robot has been loaded --> calls this->resize()
    // At least one update has been done on the task

    if(!this->robot()->isInitialized())
    {
        throw std::runtime_error("Robot is not initialised");
    }

    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = this->robot()->getBaseFrame();
    }

    if(base_ref_frame_ == this->robot()->getBaseFrame())
    {
        jacobian_ = this->robot()->getFrameFreeFloatingJacobian(control_frame_);
    }
    else
    {
        const unsigned int dof = this->robot()->getNrOfDegreesOfFreedom();
        jacobian_.block(0,6,6,dof) = this->robot()->getRelativeJacobian(base_ref_frame_,control_frame_);
    }
    jacobian_transpose_ = jacobian_.transpose();
}

void Wrench::resize()
{
    // MutexLock lock(this->mutex);

    int fulldim = this->robot()->getConfigurationSpaceDimension();

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
    
}