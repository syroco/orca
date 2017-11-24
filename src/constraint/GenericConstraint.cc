#include <orca/constraint/GenericConstraint.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::math;

GenericConstraint::GenericConstraint(ControlVariable control_var)
: TaskCommon(control_var)
{

}

void GenericConstraint::print() const
{
    MutexLock lock(mutex);

    std::cout << "[" << TaskCommon::getName() << "]" << '\n';
    std::cout << " - Size " << getSize() << '\n';
    std::cout << " - Variable  " << getControlVariable() << '\n';

    getConstraintFunction().print();

    std::cout << " - isInitialized        " << isInitialized() << '\n';
    std::cout << " - isActivated          " << isActivated() << '\n';
    std::cout << " - isInsertedInProblem  " << isInsertedInProblem() << '\n';
}

GenericConstraint::~GenericConstraint()
{
    removeFromProblem();
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

void GenericConstraint::addInRegister()
{
    OptimisationVector().addInRegister(this);
}

void GenericConstraint::removeFromRegister()
{
    OptimisationVector().removeFromRegister(this);
}

void GenericConstraint::update()
{
    MutexTryLock lock(mutex);

    if(!lock.isSuccessful())
    {
        LOG_DEBUG << "[" << TaskCommon::getName() << "] " << "Mutex is locked, skipping updating";
        return;
    }
    
    // A task is considered initialised when 
    // Robot has been loaded --> calls this->resize()
    // At least one update has been done on the task
    
    setInitialized(robot().isInitialized());

    if(!isActivated())
    {
        constraint_function_.reset();
    }

    if(isInitialized())
    {
        updateConstraintFunction();
    }
    else
    {
        LOG_ERROR << "Calling update(), but robot is not initialized";
    }
}
