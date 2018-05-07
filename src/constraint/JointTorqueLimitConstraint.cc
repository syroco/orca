#include "orca/constraint/JointTorqueLimitConstraint.h"
using namespace orca::constraint;
using namespace orca::optim;

JointTorqueLimitConstraint::JointTorqueLimitConstraint(const std::string& name)
: JointLimitConstraint(name,ControlVariable::JointSpaceTorque)
{}
