#include <orca/task/WrenchTask.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::task;
using namespace orca::optim;

WrenchTask::WrenchTask()
: GenericTask(ControlVariable::ExternalWrench)
{
    wrench_des_.setZero();
    dt_ = 0;
}

void WrenchTask::setBaseFrame(const std::string& base_ref_frame)
{
    wrench_.setBaseFrame(base_ref_frame);
}

void WrenchTask::setControlFrame(const std::string& control_frame)
{
    wrench_.setControlFrame(control_frame);
}

const std::string& WrenchTask::getBaseFrame() const
{
    return wrench_.getBaseFrame();
}

const std::string& WrenchTask::getControlFrame() const
{
    return wrench_.getControlFrame();
}

void WrenchTask::setDesired(const Vector6d& wrench_des)
{
    MutexLock lock(mutex);
    
    wrench_des_ = wrench_des;
}

void WrenchTask::setIntegralDt(double dt)
{
    MutexLock lock(mutex);
    
    if(dt>0)
        dt_ = dt;
    else
        LOG_ERROR << "dt has to be >=0";
}

void WrenchTask::updateAffineFunction()
{
    integral_error_ += dt_ * ( current_wrench_ - wrench_des_ );
    // saturate integral_term
    integral_error_.cwiseMin(   WindupLimit() );
    integral_error_.cwiseMax( - WindupLimit() );

    f() = ( - (P().asDiagonal() * ( current_wrench_ - wrench_des_ ) +  I().asDiagonal() * integral_error_) );
}

void WrenchTask::resize()
{
    MutexLock lock(mutex);
    
    int fulldim = OptimisationVector().ConfigurationSpaceDimension(); // ndof + 6
    EuclidianNorm().resize(6,fulldim);
    E().setIdentity();
}
