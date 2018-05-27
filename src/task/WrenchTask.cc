#include "orca/task/WrenchTask.h"
#include "orca/optim/ControlVariable.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;

WrenchTask::WrenchTask(const std::string& name)
: GenericTask(name,ControlVariable::ExternalWrench)
, pid_(std::make_shared<PIDController>(6))
{
    wrench_des_.setZero();
}

void WrenchTask::setDesired(const Vector6d& wrench_des)
{
    wrench_des_ = wrench_des;
}

void WrenchTask::setBaseFrame(const std::string& base_ref_frame)
{
    this->wrench()->setBaseFrame(base_ref_frame);
}

void WrenchTask::setControlFrame(const std::string& control_frame)
{
    this->wrench()->setControlFrame(control_frame);
}

const std::string& WrenchTask::getBaseFrame() const
{
    return this->getWrench()->getBaseFrame();
}

const std::string& WrenchTask::getControlFrame() const
{
    return this->getWrench()->getControlFrame();
}

void WrenchTask::setCurrentWrenchValue(const Vector6d& current_wrench_from_ft_sensor)
{
    this->wrench()->setCurrentValue(current_wrench_from_ft_sensor);
}

std::shared_ptr<PIDController> WrenchTask::pid()
{
    return pid_;
}
void WrenchTask::onActivation()
{
    wrench_des_.setZero();
}

void WrenchTask::onUpdateAffineFunction(double current_time, double dt)
{
    wrench_->update(current_time,dt);
    f() = - pid_->computeCommand( wrench_->getCurrentValue() - wrench_des_ , dt);
}

void WrenchTask::onResize()
{
    const int fulldim = this->robot()->getConfigurationSpaceDimension(); // ndof + 6
    euclidianNorm().resize(6,fulldim);
    E().setIdentity();
}
