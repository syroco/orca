#include "orca/task/WrenchTask.h"
#include "orca/optim/ControlVariable.h"

namespace orca
{
namespace task
{
using namespace orca::math;
using namespace orca::optim;
using namespace orca::common;

WrenchTask::WrenchTask(const std::string& name)
: GenericTask(name,ControlVariable::ExternalWrench)
{
    // NOTE : ExternalWrench objects creates a wrench required parameter
    this->addParameter("desired_wrench",&wrench_des_);
    this->addParameter("feedforward",&feedforward_,ParamPolicy::Optional);
    this->addParameter("pid",&pid_);
}

void WrenchTask::setDesiredWrench(const std::array<double,6>& wrench_at_control_frame)
{
    setDesiredWrench(Vector6d::Map(wrench_at_control_frame.data(),6));
}

void WrenchTask::setDesiredWrench(const Vector6d& wrench_des)
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

void WrenchTask::setFeedforward(const Vector6d& feedforward_wrench)
{
    feedforward_ = feedforward_wrench;
}

const Vector6d & WrenchTask::getFeedforward() const
{
    return feedforward_.get();
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
    if(!feedforward_.isSet())
        feedforward_ = Vector6d::Zero();
    if(!wrench_des_.isSet())
        wrench_des_ = Vector6d::Zero();
}

void WrenchTask::onUpdateAffineFunction(double current_time, double dt)
{
    f() = - feedforward_.get() - pid_.get()->computeCommand( wrench()->getCurrentValue() - wrench_des_.get() , dt);
}

void WrenchTask::onResize()
{
    euclidianNorm().resize(6,6);
    E().setIdentity();
    pid_.get()->resize(6);
}

} // namespace task
} // namespace orca

ORCA_REGISTER_CLASS(orca::task::WrenchTask)
