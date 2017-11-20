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
