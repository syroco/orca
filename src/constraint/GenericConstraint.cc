#include "orca/constraint/GenericConstraint.h"


using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::math;
using namespace orca::common;

GenericConstraint::GenericConstraint(const std::string& name,ControlVariable control_var)
: TaskBase(name,control_var)
{
    this->setRampDuration(0);
}

GenericConstraint::~GenericConstraint()
{

}

void GenericConstraint::print() const
{
    TaskBase::print();
    getConstraintFunction().print();
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
    return constraint_function_;
}

void GenericConstraint::onCompute(double current_time, double dt)
{
    onUpdateConstraintFunction(current_time,dt);
}
