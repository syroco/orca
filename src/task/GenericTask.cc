#include "orca/task/GenericTask.h"
#include "orca/optim/ControlVariable.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::math;
using namespace orca::common;

GenericTask::GenericTask(ControlVariable control_var)
: TaskBase(control_var)
{
    weight_ = 1.0;
}

void GenericTask::print() const
{
    MutexLock lock(this->mutex);
    
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
}

double GenericTask::getWeight() const
{
    MutexLock lock(this->mutex);

    return weight_;
}

void GenericTask::setWeight(double weight)
{
    MutexLock lock(this->mutex);

    weight_ = weight;
}

Size GenericTask::getSize() const
{
    MutexLock lock(this->mutex);

    return euclidian_norm_.getSize();
}

int GenericTask::cols() const
{
    MutexLock lock(this->mutex);

    return euclidian_norm_.cols();
}

int GenericTask::rows() const
{
    MutexLock lock(this->mutex);

    return euclidian_norm_.rows();
}

const WeightedEuclidianNormFunction::QuadraticCost& GenericTask::getQuadraticCost() const
{
    MutexLock lock(this->mutex);

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
    MutexLock lock(this->mutex);

    return euclidian_norm_.getA();
}

const Eigen::VectorXd& GenericTask::getf() const
{
    MutexLock lock(this->mutex);

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
    MutexTryLock lock(mutex);
    
    if(!lock.isSuccessful())
    {
        //LOG_VERBOSE << "[" << TaskBase::getName() << "] " << "Mutex is locked, skipping updating";
        return;
    }
    
    // A task is considered initialised when 
    // Robot has been loaded --> calls this->resize()
    // At least one update has been done on the task
    
    setInitialized(robot()->isInitialized());

    this->updateAffineFunction();
    this->updateQuadraticCost();
}

void GenericTask::updateQuadraticCost()
{
    euclidian_norm_.computeQuadraticCost();
}
