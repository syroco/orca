#include <orca/constraint/JointAccelerationLimitConstraint.h>
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::robot;

JointAccelerationLimitConstraint::JointAccelerationLimitConstraint()
: JointLimitConstraint(ControlVariable::JointSpaceAcceleration)
{}
