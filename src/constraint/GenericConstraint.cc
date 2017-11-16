#include <orca/constraint/GenericConstraint.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::math;

GenericConstraint::GenericConstraint(ControlVariable control_var)
: TaskCommon(control_var)
{
    this->activate();
}

void GenericConstraint::insertInProblem()
{
    if(!registered_)
    {
        OptimisationVector().addInRegister(this);
        registered_ = true;
    }
}

void GenericConstraint::removeFromProblem()
{
    if(registered_)
    {
        OptimisationVector().removeFromRegister(this);
        registered_ = false;
    }
}

GenericConstraint::~GenericConstraint()
{
    if(registered_)
    {
        removeFromProblem();
    }
}

void GenericConstraint::activate()
{
    if(is_activated_)
        std::cerr << "Contact already activated " << std::endl;
    is_activated_ = true;
}

void GenericConstraint::disactivate()
{
    if(!is_activated_)
        std::cerr << "Contact already disactivated " << std::endl;
    is_activated_ = false;
}

bool GenericConstraint::isActivated() const
{
    return is_activated_;
}

Size GenericConstraint::getSize() const
{
    return constraint_function_.getSize();
}

int GenericConstraint::rows() const
{
    return constraint_function_.rows();
}

int GenericConstraint::cols() const
{
    return constraint_function_.cols();
}

const Eigen::VectorXd& GenericConstraint::getLowerBound() const
{
    return constraint_function_.getLowerBound();
}

const Eigen::VectorXd& GenericConstraint::getUpperBound() const
{
    return constraint_function_.getUpperBound();
}

const Eigen::MatrixXd& GenericConstraint::getConstraintMatrix() const
{
    return constraint_function_.getConstraintMatrix();
}

void GenericConstraint::setConstraintMatrix(const Eigen::MatrixXd& newC)
{
    constraint_function_.constraintMatrix() = newC;
}

void GenericConstraint::setLowerBound(const Eigen::VectorXd& low)
{
    if(low.size() == constraint_function_.rows())
    {
        constraint_function_.lowerBound() = low;
    }
    else
    {
        LOG_ERROR << "Vector size should be " << constraint_function_.rows() << ", but you provided " << low.size();
    }
}

void GenericConstraint::setUpperBound(const Eigen::VectorXd& up)
{
    if(up.size() == constraint_function_.rows())
    {
        constraint_function_.upperBound() = up;
    }
    else
    {
        LOG_ERROR << "Vector size should be " << constraint_function_.rows() << ", but you provided " << up.size();
    }
}

Eigen::MatrixXd& GenericConstraint::constraintMatrix()
{
    return constraint_function_.constraintMatrix();
}

ConstraintFunction& GenericConstraint::constraintFunction()
{
    return constraint_function_;
}

const ConstraintFunction& GenericConstraint::getConstraintFunction() const
{
    return constraint_function_;
}
