#include "orca/constraint/EqualityConstraint.h"

namespace orca
{
namespace constraint
{
using namespace orca::optim;

EqualityConstraint::EqualityConstraint(const std::string& name,ControlVariable control_var)
: GenericConstraint(name,control_var)
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
    return GenericConstraint::upperBound();
}

const Eigen::VectorXd& EqualityConstraint::getBound() const
{
    return GenericConstraint::getUpperBound();
}

void EqualityConstraint::setBound(const Eigen::VectorXd& newBound)
{

    GenericConstraint::setUpperBound(newBound);
    GenericConstraint::setLowerBound(newBound);
}

} // namespace constraint
} // namespace orca
