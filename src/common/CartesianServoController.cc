#include "orca/common/CartesianServoController.h"
#include "orca/optim/ControlVariable.h"

using namespace orca::common;
using namespace orca::optim;

CartesianServoController::CartesianServoController(const std::string& name)
: TaskBase(name,ControlVariable::None)
{

}

void CartesianServoController::setBaseFrame(const std::string& base_ref_frame)
{
    LOG_INFO << "Setting reference frame to \'" << base_ref_frame  << "\'";
    base_ref_frame_ = base_ref_frame;
}

void CartesianServoController::setControlFrame(const std::string& control_frame)
{
    LOG_INFO << "Setting control frame to \'" << control_frame << "\'";
    control_frame_ = control_frame;
}

const std::string& CartesianServoController::getBaseFrame() const
{
    return base_ref_frame_;
}

const std::string& CartesianServoController::getControlFrame() const
{
    return control_frame_;
}
