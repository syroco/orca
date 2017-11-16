#include <orca/constraint/JointTorqueLimitConstraint.h>
using namespace orca::constraint;
using namespace orca::optim;

JointTorqueLimitConstraint::JointTorqueLimitConstraint()
: JointLimitConstraint(ControlVariable::JointSpaceTorque)
{}
