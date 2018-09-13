#include "orca/constraint/JointLimitConstraint.h"
#include "orca/optim/ControlVariable.h"
#include "orca/math/Utils.h"
#include "orca/utils/Utils.h"

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::common;
using namespace orca::utils;

JointLimitConstraint::JointLimitConstraint(const std::string& name,ControlVariable control_var)
: GenericConstraint(name,control_var)
{
    this->addParameter("upper_limit",&max_);
    this->addParameter("lower_limit",&min_);
}

void JointLimitConstraint::setLimits(const Eigen::VectorXd& min, const Eigen::VectorXd& max)
{
    assertSize(min,min_.get());
    assertSize(max,max_.get());
    min_ = min;
    max_ = max;
}

void JointLimitConstraint::onUpdateConstraintFunction(double current_time, double dt)
{
    constraintFunction().lowerBound() = min_.get() ;
    constraintFunction().upperBound() = max_.get() ;
}

Eigen::VectorXd& JointLimitConstraint::minLimit()
{
    return min_.get();
}
Eigen::VectorXd& JointLimitConstraint::maxLimit()
{
    return max_.get();
}

void JointLimitConstraint::onResize()
{
    const int dim = this->robot()->getNrOfDegreesOfFreedom();

    if(min_.get().size() != dim || max_.get().size() != dim)
    {
        constraintFunction().resize(dim,dim);
        constraintFunction().constraintMatrix().setIdentity();

        min_.get().conservativeResize(dim);
        max_.get().conservativeResize(dim);

        math::setToLowest(min_.get());
        math::setToHighest(max_.get());
    }
}
