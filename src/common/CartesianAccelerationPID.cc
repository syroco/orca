#include "orca/common/CartesianAccelerationPID.h"

using namespace orca::common;
using namespace orca::optim;

CartesianAccelerationPID::CartesianAccelerationPID(const std::string& name)
: TaskBase(name,ControlVariable::None)
{

}

const Eigen::Matrix4d& CartesianAccelerationPID::getCartesianPositionRef() const
{
    return cart_pos_des_;
}
const Vector6d& CartesianAccelerationPID::getCartesianVelocityRef() const
{
    return cart_vel_des_;
}
const Vector6d& CartesianAccelerationPID::getCartesianAccelerationRef() const
{
    return cart_acc_des_;
}

void CartesianAccelerationPID::resize()
{
    cart_pos_curr_.setZero();
    cart_pos_des_.setZero();
    cart_acc_cmd_.setZero();
    cart_acc_bias_.setZero();
    cart_acc_des_.setZero();
    cart_vel_des_.setZero();
    cart_pos_err_.setZero();
    cart_vel_err_.setZero();
    cart_vel_curr_.setZero();
}

void CartesianAccelerationPID::print() const
{
    std::cout << "Cartesian Acceleration PID" << '\n';
    std::cout << "Position ref : " << '\n';
    std::cout << getCartesianPositionRef() << '\n';
    std::cout << "Velocity ref : " << '\n';
    std::cout << getCartesianVelocityRef() << '\n';
    std::cout << "Acceleration ref : " << '\n';
    std::cout << getCartesianAccelerationRef() << '\n';
}

void CartesianAccelerationPID::setBaseFrame(const std::string& base_ref_frame)
{
    base_ref_frame_ = base_ref_frame;
}

void CartesianAccelerationPID::setControlFrame(const std::string& control_frame)
{
    control_frame_ = control_frame;
}

void CartesianAccelerationPID::setDesired(const Eigen::Matrix4d& cartesian_position_traj
                                        , const Vector6d& cartesian_velocity_traj
                                        , const Vector6d& cartesian_acceleration_traj)
{
    cart_pos_des_ = cartesian_position_traj;
    cart_vel_des_ = cartesian_velocity_traj;
    cart_acc_des_ = cartesian_acceleration_traj;
}

const std::string& CartesianAccelerationPID::getBaseFrame() const
{
    return base_ref_frame_;
}

const std::string& CartesianAccelerationPID::getControlFrame() const
{
    return control_frame_;
}

PIDController<6>& CartesianAccelerationPID::pid()
{
    return pid_;
}

void CartesianAccelerationPID::update(double current_time, double dt)
{
    // If no frame has been set before, use the default Floating Base.
    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot()->getBaseFrame();
    }

    // Compute Cartesian Position Error
    cart_pos_curr_ = robot()->getRelativeTransform(base_ref_frame_,control_frame_);
    cart_pos_err_ = math::diffTransform(cart_pos_curr_ , cart_pos_des_);

    // Compute Cartesian Velocity Error
    cart_vel_curr_ = robot()->getFrameVel(control_frame_);
    cart_vel_err_ = cart_vel_des_ - cart_vel_curr_;

    // Compute Cartesian Acceleration Command
    cart_acc_cmd_ = cart_acc_des_ + pid_.computeCommand( cart_pos_err_ , cart_vel_err_ , dt);
}

const Vector6d& CartesianAccelerationPID::getCommand() const
{
    return cart_acc_cmd_;
}
