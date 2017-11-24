#include <orca/common/Wrench.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::common;
using namespace orca::optim;

Wrench::Wrench()
: TaskCommon(ControlVariable::ExternalWrench)
{

}


void Wrench::setCurrent(const Eigen::Matrix<double,6,1>& current_wrench_from_ft_sensor)
{
    MutexLock lock(mutex);

    current_wrench_ = current_wrench_from_ft_sensor;
}

const Eigen::Matrix<double,6,1>& Wrench::getCurrent()
{
    MutexLock lock(mutex);

    return current_wrench_;
}

void Wrench::addInRegister()
{
    OptimisationVector().addInRegister(this);
}

void Wrench::removeFromRegister()
{
    OptimisationVector().removeFromRegister(this);
}

void Wrench::setBaseFrame(const std::string& base_ref_frame)
{
    MutexLock lock(mutex);

    if(robot().frameExists(base_ref_frame))
        base_ref_frame_ = base_ref_frame;
    else
        LOG_ERROR << "Could not set base frame to " << base_ref_frame;
}

void Wrench::setControlFrame(const std::string& control_frame)
{
    MutexLock lock(mutex);

    if(robot().frameExists(control_frame))
        control_frame_ = control_frame;
    else
        LOG_ERROR << "Could not set control frame to " << control_frame;
}

const std::string& Wrench::getBaseFrame() const
{
    MutexLock lock(mutex);

    return base_ref_frame_;
}

const std::string& Wrench::getControlFrame() const
{
    MutexLock lock(mutex);

    return control_frame_;
}

const Eigen::MatrixXd& Wrench::getJacobianTranspose() const
{
    MutexLock lock(mutex);
    if(!isActivated())
        return zero_;
    return jacobian_transpose_;
}

const Eigen::MatrixXd& Wrench::getJacobian() const
{
    MutexLock lock(mutex);

    return jacobian_;
}

void Wrench::update()
{
    MutexLock lock(mutex);
    
    // A task is considered initialised when 
    // Robot has been loaded --> calls this->resize()
    // At least one update has been done on the task
    
    setInitialized(robot().isInitialized());

    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot().getBaseFrame();
    }

    if(base_ref_frame_ == robot().getBaseFrame())
    {
        jacobian_ = robot().getFrameFreeFloatingJacobian(control_frame_);
    }
    else
    {
        const unsigned int dof = robot().getNrOfDegreesOfFreedom();
        jacobian_.block(0,6,6,dof) = robot().getRelativeJacobian(base_ref_frame_,control_frame_);
    }
    jacobian_transpose_ = jacobian_.transpose();
}

void Wrench::resize()
{
    MutexLock lock(mutex);

    int fulldim = OptimisationVector().configurationSpaceDimension(); // ndof + 6

    if(jacobian_transpose_.rows() != fulldim || jacobian_transpose_.cols() != 6)
    {
        current_wrench_.setZero();
        jacobian_transpose_.setZero(fulldim,6);
        zero_.setZero(fulldim,6);
        jacobian_.setZero(6,fulldim);
    }
}
