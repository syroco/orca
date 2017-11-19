#include <orca/common/Wrench.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::common;
using namespace orca::optim;

Wrench::Wrench()
: TaskCommon(ControlVariable::ExternalWrench)
{

}

void Wrench::activate()
{
    MutexLock lock(mutex);

    if(is_activated_)
        std::cerr << "Contact already activated " << std::endl;
    is_activated_ = true;
}

void Wrench::desactivate()
{
    MutexLock lock(mutex);

    if(!is_activated_)
        std::cerr << "Contact already desactivated " << std::endl;
    is_activated_ = false;
}

bool Wrench::isActivated() const
{
    MutexLock lock(mutex);
    
    return is_activated_;
}

void Wrench::insertInProblem()
{
    OptimisationVector().addInRegister(this);
}

void Wrench::removeFromProblem()
{
    OptimisationVector().removeFromRegister(this);
}

void Wrench::setBaseFrame(const std::string& base_ref_frame)
{
    MutexLock lock(mutex);

    if(robot().frameExists(base_ref_frame))
        base_ref_frame_ = base_ref_frame;
    else
        std::cerr << "Could not set base frame to " << base_ref_frame;
}

void Wrench::setControlFrame(const std::string& control_frame)
{
    MutexLock lock(mutex);

    if(robot().frameExists(control_frame))
        control_frame_ = control_frame;
    else
        std::cerr << "Could not set control frame to " << control_frame;
}

const std::string& Wrench::getBaseFrame() const
{
    MutexLock lock(mutex);

    return base_ref_frame_;
}

const std::string& Wrench::getControlFrame() const
{
    MutexLock lock(mutex);

    return control_frame_;
}

const Eigen::MatrixXd& Wrench::getJacobianTranspose() const
{
    MutexLock lock(mutex);
    if(!is_activated_)
        return zero_;
    return jacobian_transpose_;
}

const Eigen::MatrixXd& Wrench::getJacobian() const
{
    MutexLock lock(mutex);

    return jacobian_;
}

void Wrench::update()
{
    MutexLock lock(mutex);

    if(base_ref_frame_.empty())
    {
        base_ref_frame_ = robot().getBaseFrame();
    }

    if(base_ref_frame_ == robot().getBaseFrame())
    {
        robot().kinDynComp.getFrameFreeFloatingJacobian(control_frame_,robotData().idynJacobianFb);
        jacobian_ = iDynTree::toEigen(robotData().idynJacobianFb);
    }
    else
    {
        const int dof = robot().getNrOfDegreesOfFreedom();
        robot().kinDynComp.getRelativeJacobian(robot().kinDynComp.getFrameIndex(base_ref_frame_)
                                            ,robot().kinDynComp.getFrameIndex(control_frame_)
                                            ,robotData().idynJacobian);
        jacobian_.block(0,6,6,dof) = iDynTree::toEigen(robotData().idynJacobian);
    }
    jacobian_transpose_ = jacobian_.transpose();
}

void Wrench::resize()
{
    MutexLock lock(mutex);

    int fulldim = OptimisationVector().ConfigurationSpaceDimension(); // ndof + 6

    if(jacobian_transpose_.rows() != fulldim || jacobian_transpose_.cols() != 6)
    {
        jacobian_transpose_.setZero(fulldim,6);
        zero_.setZero(fulldim,6);
        jacobian_.setZero(6,fulldim);
    }
}
