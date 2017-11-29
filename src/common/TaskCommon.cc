#include <orca/common/TaskCommon.h>

#include <orca/optim/OptimisationVector.h>
#include <orca/util/Utils.h>
using namespace orca::common;
using namespace orca::optim;
using namespace orca::robot;

TaskCommon::TaskCommon(ControlVariable control_var)
: control_var_(control_var)
, robot_(std::make_shared<RobotDynTree>())
{
    OptimisationVector().declareObject(this);
    this->activate();
}

TaskCommon::~TaskCommon()
{
    OptimisationVector().removeObject(this);
}

void TaskCommon::setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    MutexLock lock(mutex);

    robot_->setRobotState(world_H_base,jointPos,baseVel,jointVel,gravity);
}

void TaskCommon::setRobotModel(std::shared_ptr<RobotDynTree> robot)
{
    MutexLock lock(mutex);

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

bool TaskCommon::loadRobotModel(const std::string& file_url)
{
    MutexLock lock(mutex);

    if(!robot_->loadModelFromFile(file_url))
    {
        throw std::runtime_error("Could not load robot model");
    }
    this->resize();
    return true;
}

ControlVariable TaskCommon::getControlVariable() const
{
    return control_var_;
}

void TaskCommon::setName(const std::string& name)
{
    MutexLock lock(mutex);

    name_ = name;
}

const std::string& TaskCommon::getName() const
{
    MutexLock lock(mutex);

    return name_;
}

RobotDynTree& TaskCommon::robot()
{
    return *robot_.get();
}

std::shared_ptr<RobotDynTree> TaskCommon::robotPtr()
{
    return robot_;
}

bool TaskCommon::activate()
{
    MutexLock lock(mutex);

    if(is_activated_)
    {
        LOG_WARNING << "[" << TaskCommon::getName() << "] " << "Already activated";
        return true;
    }
    else
    {
        if(true || isInitialized())
        {
            LOG_INFO << "[" << TaskCommon::getName() << "] " << "Activating";
            is_activated_ = true;
            return true;
        }
        else
        {
            LOG_WARNING << "[" << TaskCommon::getName() << "] " << "Cannot activate constraint, as robot model is not initialized";
        }
    }
    return false;
}

bool TaskCommon::desactivate()
{
    MutexLock lock(mutex);

    if(!is_activated_)
    {
        LOG_ERROR << "[" << TaskCommon::getName() << "] " << "Contact already desactivated ";
    }
    else
    {
        LOG_INFO << "[" << TaskCommon::getName() << "] " << "Desactivating";
        is_activated_ = false;
    }
    return true;
}

bool TaskCommon::isActivated() const
{
    MutexLock lock(mutex);

    return is_activated_;
}

bool TaskCommon::isInsertedInProblem() const
{
    MutexLock lock(mutex);

    return is_inserted_;
}

void TaskCommon::setInitialized(bool isinit)
{
    is_initialized_ = isinit;
}

bool TaskCommon::isInitialized() const
{
    MutexLock lock(mutex);
    
    return is_initialized_;
}

bool TaskCommon::insertInProblem()
{
    MutexLock lock(mutex);

    if(!is_inserted_)
    {
        if(isInitialized())
        {
            LOG_INFO << "[" << TaskCommon::getName() << "] " << "Inserting in problem";
            addInRegister();
            is_inserted_ = true;
            return true;
        }
        else
        {
            LOG_WARNING << "[" << TaskCommon::getName() << "] " << "Could not insert in problem as robot is not initialized (urdf loaded + received first state)";
            return false;
        }
    }
    else
    {
        LOG_WARNING << "[" << TaskCommon::getName() << "] " << "Already inserted";
    }
    return true;
}

bool TaskCommon::removeFromProblem()
{
    MutexLock lock(mutex);

    if(is_inserted_)
    {
        removeFromRegister();
        LOG_INFO << "[" << TaskCommon::getName() << "] " << "Removing from problem";
        is_inserted_ = false;
    }
    else
    {
        LOG_WARNING << "[" << TaskCommon::getName() << "] " << "Not inserted, so not removing anything";
    }
    return true;
}