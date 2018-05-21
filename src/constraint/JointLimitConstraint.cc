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
{}

void JointLimitConstraint::setLimits(const Eigen::VectorXd& min, const Eigen::VectorXd& max)
{
    assertSize(min,min_);
    assertSize(max,max_);
    min_ = min;
    max_ = max;
}

void JointLimitConstraint::onStart()
{
    
}

void JointLimitConstraint::onUpdateConstraintFunction(double current_time, double dt)
{
    constraintFunction().lowerBound() = min_ ;
    constraintFunction().upperBound() = max_ ;
}

Eigen::VectorXd& JointLimitConstraint::minLimit()
{
    return min_;
}
Eigen::VectorXd& JointLimitConstraint::maxLimit()
{
    return max_;
}

void JointLimitConstraint::onResize()
{
    const int dim = this->robot()->getNrOfDegreesOfFreedom();

    if(min_.size() != dim || max_.size() != dim)
    {
        constraintFunction().resize(dim,dim);
        constraintFunction().constraintMatrix().setIdentity();

        min_.conservativeResize(dim);
        max_.conservativeResize(dim);

        math::setToLowest(min_);
        math::setToHighest(max_);
    }
}
