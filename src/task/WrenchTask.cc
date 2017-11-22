#include <orca/task/WrenchTask.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;

WrenchTask::WrenchTask()
: GenericTask(ControlVariable::ExternalWrench)
, pid_(this->mutex)
{
    wrench_des_.setZero();
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

void WrenchTask::setCurrent(const Vector6d& wrench_curr)
{
    MutexLock lock(mutex);
    
    wrench_.setCurrent(wrench_curr);
}

PIDController<6>& WrenchTask::pid()
{
    return pid_;
}

void WrenchTask::setDt(double dt)
{
    pid_.setDt(dt);
}

void WrenchTask::updateAffineFunction()
{
    f() = - pid_.computeCommand( wrench_.getCurrent() - wrench_des_ );
}

void WrenchTask::resize()
{
    MutexLock lock(mutex);
    
    int fulldim = OptimisationVector().configurationSpaceDimension(); // ndof + 6
    EuclidianNorm().resize(6,fulldim);
    E().setIdentity();
}
