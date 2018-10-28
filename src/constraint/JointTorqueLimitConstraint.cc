#include "orca/constraint/JointTorqueLimitConstraint.h"

namespace orca
{
namespace constraint
{
using namespace orca::optim;

JointTorqueLimitConstraint::JointTorqueLimitConstraint(const std::string& name)
: JointLimitConstraint(name,ControlVariable::JointTorque)
{}

} // namespace constraint
} // namespace orca

ORCA_REGISTER_CLASS(orca::constraint::JointTorqueLimitConstraint)
