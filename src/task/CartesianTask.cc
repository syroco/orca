#include "orca/task/CartesianTask.h"
#include "orca/common/CartesianAccelerationPID.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;

CartesianTask::CartesianTask(const std::string& name)
: GenericTask(name,ControlVariable::GeneralisedAcceleration)
{
    setServoController(std::make_shared<CartesianAccelerationPID>(name + "_CartPID-EE"));
}

std::shared_ptr<CartesianServoController> CartesianTask::servoController()
{
    return servo_;
}

void CartesianTask::setServoController(std::shared_ptr<CartesianServoController> servo)
{
    servo_ = servo;
    this->link(servo_);
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
    servo_->setBaseFrame(base_ref_frame);
}

void CartesianTask::setControlFrame(const std::string& control_frame)
{
    control_frame_ = control_frame;
    servo_->setControlFrame(control_frame);
}

void CartesianTask::setDesired(const Vector6d& cartesian_acceleration_des)
{
    cart_acc_des_ = cartesian_acceleration_des;
}

void CartesianTask::onActivation()
{
    
}

void CartesianTask::onUpdateAffineFunction(double current_time, double dt)
{
    setDesired(servo_->getCommand());

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

void CartesianTask::onResize()
{
    const int fulldim = this->robot()->getConfigurationSpaceDimension();
    euclidianNorm().resize(6,fulldim);

    if(!servo_)
        throw std::runtime_error("No servo controller set. Use setServoController before inserting the task in the controller");

    // If no frame has been set before, use the default Floating Base.
    if(base_ref_frame_.empty())
    {
        LOG_WARNING << "Calling update but no baseFrame was set, setting it to the robot base frame " << robot()->getBaseFrame();
        setBaseFrame(robot()->getBaseFrame());
    }

    // Do not move at first
    cart_acc_des_.setZero();
}
