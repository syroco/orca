#include "orca/common/TaskBase.h"
#include "orca/utils/Utils.h"

using namespace orca::common;
using namespace orca::optim;
using namespace orca::robot;
using namespace orca::utils;

TaskBase::TaskBase(const std::string& name,ControlVariable control_var)
: control_var_(control_var)
, name_(name)
{
}

TaskBase::~TaskBase()
{

}

void TaskBase::print() const
{
    std::cout << "[" << TaskBase::getName() << "]" << '\n';
    std::cout << " - State " << getState() << '\n';
    std::cout << " - Variable  " << getControlVariable() << '\n';
    std::cout << " - hasProblem         " << hasProblem() << '\n';
    std::cout << " - hasRobot           " << hasRobot() << '\n';
    std::cout << " - isRobotInitialized " << isRobotInitialized() << '\n';
    std::cout << " - isActivated        " << isActivated() << '\n';
}

bool TaskBase::isRobotInitialized() const
{
    return hasRobot() && robot_->isInitialized();
}

void TaskBase::setRampValue(double new_val)
{
    ramp_value_ = std::min(1.,std::max(0.,new_val));
}

double TaskBase::getCurrentRampValue() const
{
    return ramp_value_;
}

void TaskBase::setRampDuration(double ramp_duration)
{
    if(ramp_duration < 0)
    {
        LOG_ERROR << "[" << TaskBase::getName() << "] " << "Ramp duration must be > 0";
        return;
    }
    ramp_duration_ = ramp_duration;
}

double TaskBase::getRampDuration() const
{
    return ramp_duration_;
}

double TaskBase::getStartTime() const
{
    return start_time_;
}

double TaskBase::getStopTime() const
{
    return stop_time_;
}

TaskBase::State TaskBase::getState() const
{
    return state_;
}

void TaskBase::setRobotModel(std::shared_ptr<RobotDynTree> robot)
{
    // Check if pointer is valid
    if(!robot)
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] "<< "Robot is null");
    }
    // Check if robot model as any joints
    if(robot->getNrOfDegreesOfFreedom() <= 0)
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] Robot does not seem to have any DOF");
    }
    // Check that we are in the Init state
    if(state_ != Init) {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] Calling setRobotModel is only valid when state is Init, but now it's " << state_);
    }
    // Check if we already have a robot
    if(robot_)
    {
        LOG_WARNING << "[" << TaskBase::getName() << "] " << "Replacing existing robot";
    }
    // Copy the new robot model
    robot_ = robot;
    if(hasProblem())
    {
        // Resize if we have all the parameters
        resize();
    }
}

bool TaskBase::rampUp(double time_since_start)
{
    return time_since_start < ramp_duration_;
}

bool TaskBase::rampDown(double time_since_stop)
{
    return time_since_stop < ramp_duration_;
}

void TaskBase::resize()
{
    // Calling the user callback
    this->onResize();
    state_ = Resized;
}

bool TaskBase::loadRobotModel(const std::string& file_url)
{
    if(!robot_->loadModelFromFile(file_url))
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] Could not load robot model");
    }
    return true;
}

ControlVariable TaskBase::getControlVariable() const
{
    return control_var_;
}

const std::string& TaskBase::getName() const
{
    return name_;
}

std::shared_ptr<RobotDynTree> TaskBase::robot()
{
    if(!robot_)
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] " << "Robot is not set");
    }
    return robot_;
}

bool TaskBase::start(double current_time)
{
    if(state_ == Resized || state_ == Stopped)
    {
        LOG_INFO << "[" << TaskBase::getName() << "] " << "Starting at time " << current_time;
        start_time_ = current_time;
        state_ = Starting;
        onStart();
        return true;
    }
    else
    {
        LOG_ERROR << "[" << TaskBase::getName() << "] " << "Could not start because state is " << state_;
        return false;
    }
}

void TaskBase::update(double current_time, double dt)
{
    //this->checkIfUpdatable();

    switch (state_)
    {
        case Starting:
        {
            if(this->rampUp(current_time - start_time_))
            {
                state_ = Running;
                LOG_DEBUG << "[" << TaskBase::getName() << "] " << "Ramping up is done, state is now " << state_;
            }
            break;
        }
        case Running:
        {
            this->onUpdate(current_time, dt);
            break;
        }
        case ShuttingDown:
        {
            if(this->rampDown(current_time - stop_time_))
            {
                state_ = Stopped;
                LOG_DEBUG << "[" << TaskBase::getName() << "] " << "Ramping down is done, state is now " << state_;
            }
            break;
        }
        default:
            //LOG_ERROR << "[" << TaskBase::getName() << "] " << "Should not be calling update when state is " << state_;
            break;
    }
}

bool TaskBase::stop(double current_time)
{
    if(state_ == Running)
    {
        LOG_INFO << "[" << TaskBase::getName() << "] " << "Stopping at time " << current_time;
        stop_time_ = current_time;
        state_ = ShuttingDown;
        onStop();
        return true;
    }
    else
    {
        LOG_ERROR << "[" << TaskBase::getName() << "] " << "Could not stop because state is " << state_;
        return false;
    }
}

bool TaskBase::activate()
{
    if(is_activated_)
    {
        LOG_WARNING << "[" << TaskBase::getName() << "] " << "Already activated";
        return true;
    }
    else
    {
        LOG_INFO << "[" << TaskBase::getName() << "] " << "Activating";
        is_activated_ = true;
        return true;
    }
    return false;
}

bool TaskBase::desactivate()
{
    if(!is_activated_)
    {
        LOG_ERROR << "[" << TaskBase::getName() << "] " << "Already desactivated";
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
    return is_activated_;
}

bool TaskBase::setProblem(std::shared_ptr<const Problem> problem)
{
    if(!problem)
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] "<< "Problem is null");
    }

    if(state_ != Init) {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] Calling setRobotModel is only valid when state is Init, but now it's " << state_);
    }

    if(hasProblem())
    {
        LOG_WARNING << "[" << TaskBase::getName() << "] " << "Problem is already set, replacing existing problem by the new one";
        return false;
    }
    this->problem_ = problem;
    if(hasRobot())
    {
        // Resize if we have all the parameters
        resize();
    }
    return true;
}

std::shared_ptr<const Problem> TaskBase::problem()
{
    if(!problem_)
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] "<< "Problem is not set");
    }
    return problem_;
}

void TaskBase::checkIfUpdatable() const
{
    if(!hasRobot())
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] " << "Robot is not loaded");
    }

    if(!robot_->isInitialized())
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] " << "Robot is not initialised (first state not set)");
    }

    if(!hasProblem())
    {
        throw std::runtime_error(Formatter() << "[" << TaskBase::getName() << "] " << "Problem is not set");
    }
}

bool TaskBase::hasProblem() const
{
    return static_cast<bool>(problem_);
}

bool TaskBase::hasRobot() const
{
    return static_cast<bool>(robot_);
}
