#include "orca/task/CartesianTask.h"
#include "orca/common/CartesianAccelerationPID.h"
#include <orca/common/Factory.h>

using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;

CartesianTask::CartesianTask(const std::string& name)
: GenericTask(name,ControlVariable::GeneralisedAcceleration)
{
    this->addParameter("desired_cartesian_acceleration",&cart_acc_des_,Optional);
    this->addParameter("servo_controller",&servo_);
    
    //setServoController(std::make_shared<CartesianAccelerationPID>(name + "_CartPID-EE"));
}

std::shared_ptr<CartesianServoController> CartesianTask::servoController()
{
    return servo_.get();
}

void CartesianTask::print() const
{
    std::cout << "[" << getName() << "]" << '\n';
    servo_.get()->print();
    getEuclidianNorm().print();
}

void CartesianTask::setServoController(std::shared_ptr<CartesianAccelerationPID> servo)
{
    servo_.set(servo);
    this->link(servo_.get());
}

const std::string& CartesianTask::getBaseFrame() const
{
    return servo_.get()->getBaseFrame();
}

const std::string& CartesianTask::getControlFrame() const
{
    return servo_.get()->getControlFrame();
}

void CartesianTask::setBaseFrame(const std::string& base_ref_frame)
{
    servo_.get()->setBaseFrame(base_ref_frame);
}

void CartesianTask::setControlFrame(const std::string& control_frame)
{
    servo_.get()->setControlFrame(control_frame);
}

void CartesianTask::setDesired(const Vector6d& cartesian_acceleration_des)
{
    cart_acc_des_ = cartesian_acceleration_des;
}

void CartesianTask::onActivation()
{
    // If no frame has been set before, use the default Floating Base.
    if(servo_.get()->getBaseFrame().empty())
    {
        setBaseFrame(robot()->getBaseFrame());
    }
    
    if(!cart_acc_des_.isSet())
    {
        // Do not move if no desired target is set
        cart_acc_des_.get().setZero();
    }
}

void CartesianTask::onUpdateAffineFunction(double current_time, double dt)
{
    cart_acc_des_ = servo_.get()->getCommand();

    if(getBaseFrame() == robot()->getBaseFrame())
    {
        E() = robot()->getFrameFreeFloatingJacobian(getControlFrame());
    }
    else
    {
        const int ndof = robot()->getNrOfDegreesOfFreedom();
        // Compute Jacobian
        E().block(0,6,6,ndof) = robot()->getRelativeJacobian(getBaseFrame(), getControlFrame());
        E().block(0,0,6,6).setZero();
    }

    // Compute dotJ * v
    cart_acc_bias_ = robot()->getFrameBiasAcc(getControlFrame());
    // b = dotJ.v - AccDes
    f() = ( cart_acc_bias_ - cart_acc_des_.get() );
}

void CartesianTask::onResize()
{
    const int fulldim = this->robot()->getConfigurationSpaceDimension();
    euclidianNorm().resize(6,fulldim);
}

ORCA_REGISTER_CLASS(orca::task::CartesianTask)
