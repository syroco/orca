#include "orca/common/TaskBase.h"
#include "orca/optim/ControlProblem.h"

using namespace orca::common;
using namespace orca::optim;
using namespace orca::robot;

TaskBase::TaskBase(ControlVariable control_var)
: control_var_(control_var)
{
    this->activate();
}

// void TaskBase::setRobotState(const Eigen::Matrix4d& world_H_base
//                 , const Eigen::VectorXd& jointPos
//                 , const Eigen::Matrix<double,6,1>& baseVel
//                 , const Eigen::VectorXd& jointVel
//                 , const Eigen::Vector3d& gravity)
// {
//     MutexLock lock(this->mutex);
// 
//     robot_->setRobotState(world_H_base,jointPos,baseVel,jointVel,gravity);
// }

void TaskBase::setRobotModel(std::shared_ptr<RobotDynTree> robot)
{
    MutexLock lock(this->mutex);

    if(robot)
    {
        if(robot->getNrOfDegreesOfFreedom() <= 0)
        {
            throw std::runtime_error("Robot does not seem to have any DOF");
        }

        robot_ = robot;
        this->resize();
    }
    else
    {
        throw std::runtime_error("Robot pointer is null");
    }
}

bool TaskBase::loadRobotModel(const std::string& file_url)
{
    MutexLock lock(this->mutex);

    if(!robot_->loadModelFromFile(file_url))
    {
        throw std::runtime_error("Could not load robot model");
    }
    this->resize();
    return true;
}

ControlVariable TaskBase::getControlVariable() const
{
    return control_var_;
}

void TaskBase::setName(const std::string& name)
{
    MutexLock lock(this->mutex);

    name_ = name;
}

const std::string& TaskBase::getName() const
{
    MutexLock lock(this->mutex);

    return name_;
}

std::shared_ptr<RobotDynTree> TaskBase::robot()
{
    return robot_;
}

bool TaskBase::activate()
{
    MutexLock lock(this->mutex);

    if(is_activated_)
    {
        LOG_WARNING << "[" << TaskBase::getName() << "] " << "Already activated";
        return true;
    }
    else
    {
        if(true || isInitialized())
        {
            LOG_INFO << "[" << TaskBase::getName() << "] " << "Activating";
            is_activated_ = true;
            return true;
        }
        else
        {
            LOG_WARNING << "[" << TaskBase::getName() << "] " << "Cannot activate constraint, as robot model is not initialized";
        }
    }
    return false;
}

bool TaskBase::desactivate()
{
    MutexLock lock(this->mutex);

    if(!is_activated_)
    {
        LOG_ERROR << "[" << TaskBase::getName() << "] " << "Contact already desactivated ";
    }
    else
    {
        LOG_INFO << "[" << TaskBase::getName() << "] " << "Desactivating";
        is_activated_ = false;
    }
    return true;
}

bool TaskBase::isActivated() const
{
    MutexLock lock(this->mutex);

    return is_activated_;
}

bool TaskBase::setProblem(std::shared_ptr<MultiObjectiveOptimisationProblem> problem)
{
    MutexLock lock(this->mutex);
    if(this->problem_)
    {
        LOG_ERROR << "[" << TaskBase::getName() << "] " << "Problem is already set";
        return false;
    }
    this->problem_ = problem_;
    return true;
}

std::shared_ptr<MultiObjectiveOptimisationProblem> TaskBase::problem()
{
    return problem_;
}

void TaskBase::setInitialized(bool isinit)
{
    is_initialized_ = isinit;
}

bool TaskBase::isInitialized() const
{
    MutexLock lock(this->mutex);
    
    return is_initialized_;
}

bool TaskBase::hasProblem() const
{
    return static_cast<bool>(problem_);
}

bool TaskBase::hasRobot() const
{
    return static_cast<bool>(robot_);
}

