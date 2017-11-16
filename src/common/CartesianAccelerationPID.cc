#include <orca/common/CartesianAccelerationPID.h>

using namespace orca::common;
using namespace orca::optim;

CartesianAccelerationPID::CartesianAccelerationPID()
: TaskCommon(ControlVariable::None)
{

}

void CartesianAccelerationPID::resize()
{

}

void CartesianAccelerationPID::setBaseFrame(const std::string& base_ref_frame)
{
    MutexLock lock(mutex);
    
    if(robot().frameExists(base_ref_frame))
        base_ref_frame_ = base_ref_frame;
}
void CartesianAccelerationPID::setControlFrame(const std::string& control_frame)
{
    MutexLock lock(mutex);
    
    if(robot().frameExists(control_frame))
        control_frame_ = control_frame;
}
void CartesianAccelerationPID::setDesired(const Eigen::Matrix4d& cartesian_position_traj
                                        , const Vector6d& cartesian_velocity_traj
                                        , const Vector6d& cartesian_acceleration_traj)
{
    MutexLock lock(mutex);
    
    cart_pos_des_ = cartesian_position_traj;
    cart_vel_des_ = cartesian_velocity_traj;
    cart_acc_des_ = cartesian_acceleration_traj;
}
const std::string & CartesianAccelerationPID::getBaseFrame() const
{
    MutexLock lock(mutex);
    
    return base_ref_frame_;
}
const std::string & CartesianAccelerationPID::getControlFrame() const
{
    MutexLock lock(mutex);
    
    return control_frame_;
}
void CartesianAccelerationPID::update()
{
    MutexLock lock(mutex);
    
    // If no frame has been set before, use the default Floating Base.
    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot().getBaseFrame();
    }
    
    // Compute Cartesian Position Error
    cart_pos_curr_ = iDynTree::toEigen(robot().kinDynComp.getRelativeTransform(base_ref_frame_,control_frame_).asHomogeneousTransform());
    cart_pos_err_ = math::diffTransform(cart_pos_curr_ , cart_pos_des_);

    // Compute Cartesian Velocity Error
    cart_vel_curr_ = iDynTree::toEigen(robot().kinDynComp.getFrameVel(control_frame_));
    cart_vel_err_ = cart_vel_des_ - cart_vel_curr_;

    // Compute Cartesian Acceleration Command
    cart_acc_cmd_ = cart_acc_des_ + P().asDiagonal() * cart_pos_err_ + D().asDiagonal() * cart_vel_err_;
}
const Vector6d& CartesianAccelerationPID::getCommand() const
{
    MutexLock lock(mutex);
    
    return cart_acc_cmd_;
}
