#include "orca/constraint/GenericConstraint.h"


using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::math;
using namespace orca::common;

GenericConstraint::GenericConstraint(ControlVariable control_var)
: TaskBase(control_var)
{

}

void GenericConstraint::print() const
{
    MutexLock lock(this->mutex);

    std::cout << "[" << TaskBase::getName() << "]" << '\n';
    std::cout << " - Size " << getSize() << '\n';
    std::cout << " - Variable  " << getControlVariable() << '\n';

    getConstraintFunction().print();

    std::cout << " - isInitialized        " << isInitialized() << '\n';
    std::cout << " - isActivated          " << isActivated() << '\n';
    //std::cout << " - isInsertedInProblem  " << isInsertedInProblem() << '\n';
}

GenericConstraint::~GenericConstraint()
{
    // This cannot be in TaskBase because when invoking the destructor 'this' is actually a taskBase
    removeFromProblem();
}

Size GenericConstraint::getSize() const
{
    MutexLock lock(this->mutex);

    return constraint_function_.getSize();
}

int GenericConstraint::rows() const
{
    MutexLock lock(this->mutex);

    return constraint_function_.rows();
}

int GenericConstraint::cols() const
{
    MutexLock lock(this->mutex);

    return constraint_function_.cols();
}

const Eigen::VectorXd& GenericConstraint::getLowerBound() const
{
    MutexLock lock(this->mutex);

    return constraint_function_.getLowerBound();
}

const Eigen::VectorXd& GenericConstraint::getUpperBound() const
{
    MutexLock lock(this->mutex);

    return constraint_function_.getUpperBound();
}

const Eigen::MatrixXd& GenericConstraint::getConstraintMatrix() const
{
    MutexLock lock(this->mutex);

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
    MutexLock lock(this->mutex);

    return constraint_function_;
}

void GenericConstraint::update()
{
    MutexTryLock lock(this->mutex);

    if(!lock.isSuccessful())
    {
        //LOG_VERBOSE << "[" << TaskBase::getName() << "] " << "Mutex is locked, skipping updating";
        return;
    }
    
    this->printStateIfErrors();
    
    if(isInitialized())
    {
        updateConstraintFunction();
    }
}
