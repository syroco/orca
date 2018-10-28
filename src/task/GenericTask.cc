#include "orca/task/GenericTask.h"
#include "orca/optim/ControlVariable.h"

namespace orca
{
namespace task
{
using namespace orca::optim;
using namespace orca::math;
using namespace orca::common;
using namespace orca::utils;

GenericTask::GenericTask(const std::string& name,ControlVariable control_var)
: TaskBase(name,control_var)
{
    this->setRampDuration(0);
    this->addParameter("weight",&weight_);
}

GenericTask::~GenericTask()
{

}

void GenericTask::print() const
{
    TaskBase::print();
    getEuclidianNorm().print();
    std::cout << " - Weight " << weight_.get() << '\n';
    std::cout << " - Ramp   " << getCurrentRampValue() << '\n';
    std::cout << " - Ramp duration " << getRampDuration() << '\n';
}

double GenericTask::getWeight() const
{
    return getCurrentRampValue() * weight_.get();
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

void GenericTask::setE(const Eigen::MatrixXd& newE)
{
    return euclidian_norm_.setA(newE);
}

void GenericTask::setf(const Eigen::VectorXd& newf)
{
    return euclidian_norm_.setb(newf);
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
        setRampValue( time_since_start *( weight_.get() / getRampDuration() ) );;
        return false;
    }
}

void GenericTask::onCompute(double current_time, double dt)
{
    Size Esize_before  = Size(E());
    Size fsize_before  = Size(f());

    this->onUpdateAffineFunction(current_time, dt);

    Size Esize_after  = Size(E());
    Size fsize_after  = Size(f());

    if(Esize_before != Esize_after)
    {
        orca_throw(Formatter() << "[" << getName() << "] Matrix E() changed size during onUpdateAffineFunction, it was ("
                << Esize_before
                << ") but now its ("
                << Esize_after
                << "). Make sure your math is correct"
            );
    }
    if(fsize_before != fsize_after)
    {
        orca_throw(Formatter() << "[" << getName() << "] Vector f() changed size during onUpdateAffineFunction, it was ("
                << Esize_before
                << ") but now its ("
                << Esize_after
                << "). Make sure your math is correct"
            );
    }
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

void GenericTask::onDeactivation()
{

}

void GenericTask::computeQuadraticCost()
{
    euclidian_norm_.computeQuadraticCost();
}

} // namespace task
} // namespace orca
