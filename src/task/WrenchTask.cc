#include "orca/task/WrenchTask.h"
#include "orca/optim/ControlVariable.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;

WrenchTask::WrenchTask(const std::string& name)
: GenericTask(name,ControlVariable::ExternalWrench)
, wrench_(std::make_shared<Wrench>(name + "_wrench"))
{
    wrench_des_.setZero();
}

void WrenchTask::setDesired(const Vector6d& wrench_des)
{
    wrench_des_ = wrench_des;
}

void WrenchTask::setBaseFrame(const std::string& base_ref_frame)
{
    wrench_->setBaseFrame(base_ref_frame);
}

void WrenchTask::setControlFrame(const std::string& control_frame)
{
    wrench_->setControlFrame(control_frame);
}

const std::string& WrenchTask::getBaseFrame() const
{
    return wrench_->getBaseFrame();
}

const std::string& WrenchTask::getControlFrame() const
{
    return wrench_->getControlFrame();
}

void WrenchTask::setCurrentWrenchValue(const Vector6d& current_wrench_from_ft_sensor)
{
    wrench_->setCurrentValue(current_wrench_from_ft_sensor);
}

PIDController<6>& WrenchTask::pid()
{
    return pid_;
}
void WrenchTask::onStart()
{
    wrench_des_.setZero();
}

void WrenchTask::onUpdateAffineFunction(double current_time, double dt)
{
    wrench_->update(current_time,dt);
    f() = - pid_.computeCommand( wrench_->getCurrentValue() - wrench_des_ , dt);
}

std::shared_ptr<const Wrench> WrenchTask::getWrench() const
{
    return wrench_;
}

void WrenchTask::onResize()
{
    wrench_->resize();
    const int fulldim = this->robot()->getConfigurationSpaceDimension(); // ndof + 6
    euclidianNorm().resize(6,fulldim);
    E().setIdentity();
}
