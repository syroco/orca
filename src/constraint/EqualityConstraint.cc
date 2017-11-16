#include <orca/constraint/EqualityConstraint.h>
using namespace orca::constraint;
using namespace orca::optim;

EqualityConstraint::EqualityConstraint(ControlVariable control_var)
: GenericConstraint(control_var)
{

}

const Eigen::VectorXd& EqualityConstraint::getLowerBound() const
{
    return GenericConstraint::getUpperBound();
}

const Eigen::VectorXd& EqualityConstraint::getUpperBound() const
{
    return GenericConstraint::getUpperBound();
}

Eigen::VectorXd& EqualityConstraint::bound()
{
    return constraintFunction().upperBound();
}

const Eigen::VectorXd& EqualityConstraint::getBound() const
{
    return GenericConstraint::getUpperBound();
}
