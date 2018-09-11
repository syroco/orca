#include "orca/task/CartesianTask.h"
#include "orca/common/CartesianAccelerationPID.h"

using namespace orca::task;
using namespace orca::optim;
using namespace orca::common;

CartesianTask::CartesianTask(const std::string& name)
: GenericTask(name,ControlVariable::GeneralisedAcceleration)
{
    this->addParameter("control_frame",&control_frame_);
    this->addParameter("base_frame",&base_ref_frame_,Optional);
    this->addParameter("desired_cartesian_acceleration",&cart_acc_des_,Optional);
    this->addParameter("servo_controller",&servo_);
    
    setServoController(std::make_shared<CartesianAccelerationPID>(name + "_CartPID-EE"));
}

std::shared_ptr<CartesianServoController> CartesianTask::servoController()
{
    return servo_.get();
}

void CartesianTask::print() const
{
    std::cout << "[" << getName() << "]" << '\n';
    std::cout << " Base frame " << getBaseFrame() << '\n';
    std::cout << " Control Frame " << getControlFrame() << '\n';
    servo_.get()->print();
    getEuclidianNorm().print();
}

void CartesianTask::setServoController(std::shared_ptr<CartesianAccelerationPID> servo)
{
    servo_ = servo;
    this->link(servo_.get());
}

const std::string& CartesianTask::getBaseFrame() const
{
    return base_ref_frame_.get();
}

const std::string& CartesianTask::getControlFrame() const
{
    return control_frame_.get();
}

void CartesianTask::setBaseFrame(const std::string& base_ref_frame)
{
    base_ref_frame_ = base_ref_frame;
    servo_.get()->setBaseFrame(base_ref_frame);
}

void CartesianTask::setControlFrame(const std::string& control_frame)
{
    control_frame_ = control_frame;
    servo_.get()->setControlFrame(control_frame);
}

void CartesianTask::setDesired(const Vector6d& cartesian_acceleration_des)
{
    cart_acc_des_ = cartesian_acceleration_des;
}

void CartesianTask::onActivation()
{
    if(!cart_acc_des_.isSet())
    {
        // Do not move if no desired target is set
        cart_acc_des_.get().setZero();
    }
}

void CartesianTask::onUpdateAffineFunction(double current_time, double dt)
{
    setDesired(servo_.get()->getCommand());

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

    // If no frame has been set before, use the default Floating Base.
    if(base_ref_frame_.get().empty())
    {
        LOG_WARNING << "Calling resize but no baseFrame was set, setting it to the robot base frame " << robot()->getBaseFrame();
        setBaseFrame(robot()->getBaseFrame());
    }
}
