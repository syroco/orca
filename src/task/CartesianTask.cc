#include "orca/task/CartesianTask.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;

CartesianTask::CartesianTask(const std::string& name)
: GenericTask(name,ControlVariable::GeneralisedAcceleration)
{

}

const std::string& CartesianTask::getBaseFrame() const
{
    return base_ref_frame_;
}

const std::string& CartesianTask::getControlFrame() const
{
    return control_frame_;
}

void CartesianTask::setBaseFrame(const std::string& base_ref_frame)
{
    base_ref_frame_ = base_ref_frame;
}

void CartesianTask::setControlFrame(const std::string& control_frame)
{
    control_frame_ = control_frame;
}

void CartesianTask::setDesired(const Vector6d& cartesian_acceleration_des)
{
    cart_acc_des_ = cartesian_acceleration_des;
}


void CartesianTask::updateAffineFunction(double current_time, double dt)
{
    // If no frame has been set before, use the default Floating Base.
    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot()->getBaseFrame();
    }


    if(base_ref_frame_ == robot()->getBaseFrame())
    {
        E() = robot()->getFrameFreeFloatingJacobian(this->control_frame_);
    }
    else
    {
        const int ndof = robot()->getNrOfDegreesOfFreedom();
        // Compute Jacobian
        E().block(0,6,6,ndof) = robot()->getRelativeJacobian(this->base_ref_frame_
                                            , this->control_frame_);
        E().block(0,0,6,6).setZero();
    }

    // Compute dotJ * v
    cart_acc_bias_ = robot()->getFrameBiasAcc(this->control_frame_);
    // b = dotJ.v - AccDes
    f() = ( cart_acc_bias_ - cart_acc_des_ );
}

void CartesianTask::resize()
{
    const int fulldim = this->robot()->getConfigurationSpaceDimension();
    euclidianNorm().resize(6,fulldim);
}
