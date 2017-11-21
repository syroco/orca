#include <orca/constraint/GenericConstraint.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::math;

GenericConstraint::GenericConstraint(ControlVariable control_var)
: TaskCommon(control_var)
, is_activated_(false)
{
    this->activate();
}

void GenericConstraint::insertInProblem()
{
    MutexLock lock(mutex);

    if(!registered_)
    {
        OptimisationVector().addInRegister(shared_from_this());
        registered_ = true;
    }
}

void GenericConstraint::removeFromProblem()
{
    MutexLock lock(mutex);

    if(registered_)
    {
        OptimisationVector().removeFromRegister(shared_from_this());
        registered_ = false;
    }
}

GenericConstraint::~GenericConstraint()
{
    removeFromProblem();
}

void GenericConstraint::activate()
{
    MutexLock lock(mutex);

    if(is_activated_)
    {
        LOG_ERROR << "Contact already activated ";
    }
    else
    {
        is_activated_ = true;
    }
}

void GenericConstraint::desactivate()
{
    MutexLock lock(mutex);

    if(!is_activated_)
    {
        LOG_ERROR << "Contact already desactivated ";
    }
    else
    {
        is_activated_ = false;
    }
}

bool GenericConstraint::isActivated() const
{
    MutexLock lock(mutex);

    return is_activated_;
}

bool GenericConstraint::isInsertedInProblem() const
{
    MutexLock lock(mutex);

    return registered_;
}

Size GenericConstraint::getSize() const
{
    MutexLock lock(mutex);

    return constraint_function_.getSize();
}

int GenericConstraint::rows() const
{
    MutexLock lock(mutex);

    return constraint_function_.rows();
}

int GenericConstraint::cols() const
{
    MutexLock lock(mutex);

    return constraint_function_.cols();
}

const Eigen::VectorXd& GenericConstraint::getLowerBound() const
{
    MutexLock lock(mutex);

    return constraint_function_.getLowerBound();
}

const Eigen::VectorXd& GenericConstraint::getUpperBound() const
{
    MutexLock lock(mutex);

    return constraint_function_.getUpperBound();
}

const Eigen::MatrixXd& GenericConstraint::getConstraintMatrix() const
{
    MutexLock lock(mutex);

    return constraint_function_.getConstraintMatrix();
}

void GenericConstraint::setConstraintMatrix(const Eigen::MatrixXd& newC)
{
    constraint_function_.setConstraintMatrix(newC);
}

void GenericConstraint::setLowerBound(const Eigen::VectorXd& low)
{
    constraint_function_.setLowerBound( low );
}

void GenericConstraint::setUpperBound(const Eigen::VectorXd& up)
{
    constraint_function_.setUpperBound( up );
}

Eigen::MatrixXd& GenericConstraint::constraintMatrix()
{
    return constraint_function_.constraintMatrix();
}

Eigen::VectorXd& GenericConstraint::upperBound()
{
    return constraint_function_.upperBound();
}

Eigen::VectorXd& GenericConstraint::lowerBound()
{
    return constraint_function_.lowerBound();
}

ConstraintFunction& GenericConstraint::constraintFunction()
{
    return constraint_function_;
}

const ConstraintFunction& GenericConstraint::getConstraintFunction() const
{
    MutexLock lock(mutex);

    return constraint_function_;
}
