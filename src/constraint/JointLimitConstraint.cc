#include "orca/constraint/JointLimitConstraint.h"
#include "orca/optim/ControlVariable.h"
#include "orca/math/Utils.h"

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::common;

JointLimitConstraint::JointLimitConstraint(ControlVariable control_var)
: GenericConstraint(control_var)
{}

void JointLimitConstraint::setLimits(const Eigen::VectorXd& min, const Eigen::VectorXd& max)
{
    MutexLock lock(this->mutex);

    min_ = min;
    max_ = max;
}

void JointLimitConstraint::updateConstraintFunction()
{
    MutexLock lock(this->mutex);

    constraintFunction().lowerBound() = min_ ;
    constraintFunction().upperBound() = max_ ;
}

void JointLimitConstraint::resize()
{
    MutexLock lock(this->mutex);

    const int dim = this->robot()->getNrOfDegreesOfFreedom();

    if(min_.size() != dim || max_.size() != dim)
    {
        constraintFunction().resize(dim,dim);
        constraintFunction().constraintMatrix().setIdentity();

        min_.resize(dim);
        max_.resize(dim);

        math::setToLowest(min_);
        math::setToHighest(max_);
    }
}
