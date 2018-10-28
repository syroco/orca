#include "orca/common/CartesianServoController.h"
#include "orca/optim/ControlVariable.h"
#include "orca/utils/Utils.h"

namespace orca
{
namespace common
{

using namespace orca::optim;
using namespace orca::utils;

CartesianServoController::CartesianServoController(const std::string& name)
: TaskBase(name,ControlVariable::None)
{
    this->addParameter("base_frame",&base_ref_frame_,ParamPolicy::Optional);
    this->addParameter("control_frame",&control_frame_);
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
    return base_ref_frame_.get();
}

const std::string& CartesianServoController::getControlFrame() const
{
    return control_frame_.get();
}

} // namespace common
} // namespace orca
