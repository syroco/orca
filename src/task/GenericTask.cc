#include <orca/task/GenericTask.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::task;
using namespace orca::optim;
using namespace orca::math;

GenericTask::GenericTask(ControlVariable control_var)
: TaskCommon(control_var)
{
    weight_ = 1.0;
    registered_ = false;
}

void GenericTask::insertInProblem()
{
    if(!registered_)
    {
        OptimisationVector().addInRegister(this);
        registered_ = true;
    }
}

void GenericTask::removeFromProblem()
{
    if(registered_)
    {
        OptimisationVector().removeFromRegister(this);
        registered_ = false;
    }
}

GenericTask::~GenericTask()
{
    if(registered_)
    {
        removeFromProblem();
    }
}

int GenericTask::getWeight() const
{
    return weight_;
}

void GenericTask::setWeight(double weight)
{
    weight_ = weight;
}

Size GenericTask::getSize() const
{
    return euclidian_norm_.getSize();
}

int GenericTask::cols() const
{
    return euclidian_norm_.cols();
}

int GenericTask::rows() const
{
    return euclidian_norm_.rows();
}

const WeightedEuclidianNormFunction::QuadraticCost& GenericTask::getQuadraticCost() const
{
    return euclidian_norm_.getQuadraticCost();
}

WeightedEuclidianNormFunction& GenericTask::EuclidianNorm()
{
    return euclidian_norm_;
}

const WeightedEuclidianNormFunction& GenericTask::getEuclidianNorm() const
{
    return euclidian_norm_;
}

const Eigen::MatrixXd& GenericTask::getE() const
{
    return euclidian_norm_.getA();
}

const Eigen::VectorXd& GenericTask::getf() const
{
    return euclidian_norm_.getb();
}

Eigen::MatrixXd& GenericTask::E()
{
    return euclidian_norm_.A();
}

Eigen::VectorXd& GenericTask::f()
{
    return euclidian_norm_.b();
}

void GenericTask::update()
{
    this->updateAffineFunction();
    this->updateQuadraticCost();
}

void GenericTask::updateQuadraticCost()
{
    euclidian_norm_.computeQuadraticCost();
}

