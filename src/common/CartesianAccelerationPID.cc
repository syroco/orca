#include "orca/common/CartesianAccelerationPID.h"

using namespace orca::common;
using namespace orca::optim;

CartesianAccelerationPID::CartesianAccelerationPID(const std::string& name)
: CartesianServoController(name)
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

void CartesianAccelerationPID::onResize()
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
    std::cout << "[" << getName() << "]" << '\n';
    std::cout << " - Position ref : " << '\n';
    std::cout << getCartesianPositionRef() << '\n';
    std::cout << " - Velocity ref : " << '\n';
    std::cout << getCartesianVelocityRef() << '\n';
    std::cout << " - Acceleration ref : " << '\n';
    std::cout << getCartesianAccelerationRef() << '\n';
}

void CartesianAccelerationPID::setDesired(const Eigen::Matrix4d& cartesian_position_traj
                                        , const Vector6d& cartesian_velocity_traj
                                        , const Vector6d& cartesian_acceleration_traj)
{
    cart_pos_des_ = cartesian_position_traj;
    cart_vel_des_ = cartesian_velocity_traj;
    cart_acc_des_ = cartesian_acceleration_traj;
}

PIDController<6>& CartesianAccelerationPID::pid()
{
    return pid_;
}

void CartesianAccelerationPID::onStart()
{
    cart_pos_des_ = robot()->getRelativeTransform(getBaseFrame(),getControlFrame());
}

void CartesianAccelerationPID::onUpdate(double current_time, double dt)
{
    // If no frame has been set before, use the default Floating Base.
    if(getBaseFrame().empty())
        throw std::runtime_error("baseframe is empty");
    if(getControlFrame().empty())
        throw std::runtime_error("controlFrame is empty");

    // Compute Cartesian Position Error
    cart_pos_curr_ = robot()->getRelativeTransform(getBaseFrame(),getControlFrame());
    cart_pos_err_ = math::diffTransform(cart_pos_curr_ , cart_pos_des_);

    // Compute Cartesian Velocity Error
    cart_vel_curr_ = robot()->getFrameVel(getControlFrame());
    cart_vel_err_ = cart_vel_des_ - cart_vel_curr_;

    // Compute Cartesian Acceleration Command
    cart_acc_cmd_ = cart_acc_des_ + pid_.computeCommand( cart_pos_err_ , cart_vel_err_ , dt);
}

void CartesianAccelerationPID::onStop()
{

}

const Vector6d& CartesianAccelerationPID::getCommand() const
{
    return cart_acc_cmd_;
}
