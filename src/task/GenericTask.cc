#include "orca/task/GenericTask.h"
#include "orca/optim/ControlVariable.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::math;
using namespace orca::common;

GenericTask::GenericTask(const std::string& name,ControlVariable control_var)
: TaskBase(name,control_var)
{

}

GenericTask::~GenericTask()
{

}

void GenericTask::print() const
{
    TaskBase::print();
    getEuclidianNorm().print();
    std::cout << " - Weight " << getWeight() << '\n';
    std::cout << " - Ramp   " << getCurrentRampValue() << '\n';
}

double GenericTask::getWeight() const
{
    return getCurrentRampValue() * weight_;
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

bool GenericTask::rampUp(double time_since_start)
{
    if(time_since_start >= getRampDuration())
    {
        setRampValue( 1 );
        return true;
    }
    else
    {
        setRampValue( time_since_start *( getWeight() / getRampDuration() ) );;
        return false;
    }
}

void GenericTask::onUpdate(double current_time, double dt)
{
    this->onUpdateAffineFunction(current_time, dt);
    this->computeQuadraticCost();
}

bool GenericTask::rampDown(double time_since_start)
{
    if(time_since_start >= getRampDuration())
    {
        setRampValue( 0 );
        return true;
    }
    else
    {
        setRampValue( - time_since_start *( getWeight() / getRampDuration() ) );
        return false;
    }
}

void GenericTask::onStop()
{

}

void GenericTask::computeQuadraticCost()
{
    euclidian_norm_.computeQuadraticCost();
}
