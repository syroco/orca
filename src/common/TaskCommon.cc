#include <orca/common/TaskCommon.h>

#include <orca/optim/OptimisationVector.h>
#include <orca/util/Utils.h>
using namespace orca::common;
using namespace orca::optim;
using namespace orca::robot;

TaskCommon::TaskCommon(ControlVariable control_var)
: control_var_(control_var)
, robot_(new RobotDynTree)
{
    
}

void TaskCommon::initializeRobotData()
{
    if(robot_->getNrOfDegreesOfFreedom() > 0)
    {
        robot_data_helper_.resize(robot_->kinDynComp.model());
    }
    else
    {
        throw std::runtime_error("Could not read any DOFs");
    }
}

void TaskCommon::setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    robot_->setRobotState(world_H_base,jointPos,baseVel,jointVel,gravity);
}

void TaskCommon::setRobotModel(std::shared_ptr<RobotDynTree> robot)
{
    if(robot)
    {
        if(robot->getNrOfDegreesOfFreedom() <= 0)
        {
            throw std::runtime_error("Robot does not seem to have any DOF");
        }

        robot_ = robot;
        this->initializeRobotData();
        this->resize();
    }
    else
    {
        throw std::runtime_error("Robot pointer is null");
    }
}

bool TaskCommon::loadRobotModel(const std::string& file_url)
{
    if(!robot_->loadModelFromFile(file_url))
    {
        throw std::runtime_error("Could not load robot model");
    }
    this->initializeRobotData();
    this->resize();
    return true;
}

ControlVariable TaskCommon::getControlVariable() const
{
    return control_var_;
}

void TaskCommon::setName(const std::string& name)
{
    name_ = name;
}

const std::string& TaskCommon::getName() const
{
    return name_;
}


RobotDataHelper& TaskCommon::robotData()
{
    return robot_data_helper_;
}

RobotDynTree& TaskCommon::robot()
{
    return *robot_.get();
}

std::shared_ptr<RobotDynTree> TaskCommon::robotPtr()
{
    return robot_;
}
