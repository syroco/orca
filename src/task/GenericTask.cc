#include "orca/task/GenericTask.h"
#include "orca/optim/ControlVariable.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::math;
using namespace orca::common;

GenericTask::GenericTask(const std::string& name,ControlVariable control_var)
: TaskBase(name,control_var)
{
    weight_ = 1.0;
}

void GenericTask::print() const
{
    std::cout << "[" << TaskBase::getName() << "]" << '\n';
    std::cout << " - Weight " << getWeight() << '\n';
    std::cout << " - Size " << getSize() << '\n';
    std::cout << " - Variable  " << getControlVariable() << '\n';

    getEuclidianNorm().print();

    std::cout << " - isInitialized        " << isInitialized() << '\n';
    std::cout << " - isActivated          " << isActivated() << '\n';
    //std::cout << " - isInsertedInProblem  " << isInsertedInProblem() << '\n';
}


GenericTask::~GenericTask()
{
    // This cannot be in TaskBase because when invoking the destructor 'this' is actually a taskBase
    // removeFromProblem();
}

double GenericTask::getWeight() const
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

WeightedEuclidianNormFunction& GenericTask::euclidianNorm()
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

void GenericTask::update(double current_time, double dt)
{
//     if(!lock.isSuccessful())
//     {
//         //LOG_VERBOSE << "[" << TaskBase::getName() << "] " << "Mutex is locked, skipping updating";
//         return;
//     }

    this->printStateIfErrors();

    if(isInitialized())
    {
        this->updateAffineFunction(current_time, dt);
        this->computeQuadraticCost();
    }
}

void GenericTask::computeQuadraticCost()
{
    euclidian_norm_.computeQuadraticCost();
}
