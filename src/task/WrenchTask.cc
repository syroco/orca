#include "orca/task/WrenchTask.h"
#include "orca/optim/ControlVariable.h"

using namespace orca::math;
using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;

WrenchTask::WrenchTask(const std::string& name)
: GenericTask(name,ControlVariable::ExternalWrench)
{
    // NOTE : ExternalWrench objects creates a wrench required parameter
    wrench_des_ = Vector6d::Zero();
    this->addParameter("desired_wrench",&wrench_des_);
    this->addParameter("pid",&pid_);
}

void WrenchTask::setDesired(const std::array<double,6>& wrench_at_control_frame)
{
    setDesired(Vector6d::Map(wrench_at_control_frame.data(),6));
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

PIDController::Ptr WrenchTask::pid()
{
    return pid_.get();
}
void WrenchTask::onActivation()
{
    wrench_des_ = Vector6d::Zero();
}

void WrenchTask::onUpdateAffineFunction(double current_time, double dt)
{
    f() = - pid_.get()->computeCommand( wrench()->getCurrentValue() - wrench_des_.get() , dt);
}

void WrenchTask::onResize()
{
    euclidianNorm().resize(6,6);
    E().setIdentity();
    pid_.get()->resize(6);
}

ORCA_REGISTER_CLASS(orca::task::WrenchTask)
