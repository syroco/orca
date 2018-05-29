#include "orca/optim/Controller.h"
#include "orca/common/Wrench.h"
#include "orca/common/TaskBase.h"
#include "orca/constraint/Contact.h"
#include "orca/task/RegularisationTask.h"
#include "orca/constraint/DynamicsEquationConstraint.h"

using namespace orca;
using namespace orca::optim;

Controller::Controller(const std::string& name
    , std::shared_ptr<robot::RobotDynTree> robot
    ,ResolutionStrategy resolution_strategy
    ,QPSolver::SolverType solver_type)
: name_(name)
, robot_(robot)
, resolution_strategy_(resolution_strategy)
, solver_type_(solver_type)
{
    joint_acceleration_command_.setZero(robot_->getNrOfDegreesOfFreedom());
    joint_torque_command_.setZero(robot_->getNrOfDegreesOfFreedom());
    kkt_torques_.setZero(robot_->getNrOfDegreesOfFreedom());

    if(resolution_strategy != ResolutionStrategy::OneLevelWeighted)
    {
        throw std::runtime_error(utils::Formatter() << "Only ResolutionStrategy::OneLevelWeighted is supported for now");
    }
    insertNewProblem();
}

void Controller::print() const
{
    for(auto problem : problems_)
    {
        problem->print();
    }
}

void Controller::setPrintLevel(int level)
{
    for(auto problem : problems_)
    {
        problem->qpSolver()->setPrintLevel(level);
    }
}

const std::string& Controller::getName()
{
    return name_;
}

std::shared_ptr<robot::RobotDynTree> Controller::robot()
{
    if(!robot_)
        throw std::runtime_error(utils::Formatter() << "Robot is not set");
    return robot_;
}

void Controller::setRobotModel(std::shared_ptr<robot::RobotDynTree> robot)
{
    robot_ = robot;
}

void Controller::setUpdateCallback(std::function<void(double,double)> update_cb)
{
    this->update_cb_ = update_cb;
}

bool Controller::update(double current_time, double dt)
{
    solution_found_ = false;

    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
        {
            updateTasks(current_time,dt);
            updateConstraints(current_time,dt);
            problems_.front()->build();
            solution_found_ = problems_.front()->solve();

            if(this->update_cb_)
                this->update_cb_(current_time,dt);

            return solution_found_;
        }
        default:
            throw std::runtime_error(utils::Formatter() << "unsupported resolution strategy");
    }
}

bool Controller::solutionFound() const
{
    return solution_found_;
}

common::ReturnCode Controller::getReturnCode() const
{
    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
            return problems_.front()->getReturnCode();
        default:
            throw std::runtime_error(utils::Formatter() << "unsupported resolution strategy");
    }
}

bool Controller::addTask(std::shared_ptr<task::GenericTask> task)
{
    if(resolution_strategy_ == ResolutionStrategy::OneLevelWeighted)
    {
        task->setRobotModel(robot_);
        task->setProblem(problems_.front());
        return problems_.front()->addTask(task);
    }
    return false;
}

bool Controller::addConstraint(std::shared_ptr<constraint::GenericConstraint> cstr)
{
    if(resolution_strategy_ == ResolutionStrategy::OneLevelWeighted)
    {
        cstr->setRobotModel(robot_);
        cstr->setProblem(problems_.front());
        return problems_.front()->addConstraint(cstr);
    }
    return false;
}

const Eigen::VectorXd& Controller::getSolution()
{
    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
            return problems_.front()->getSolution();
        default:
            throw std::runtime_error(utils::Formatter() << "Unsupported resolution strategy");
    }
}

const Eigen::VectorXd& Controller::getJointTorqueCommand()
{
    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
            joint_torque_command_ = problems_.front()->getSolution(ControlVariable::JointSpaceTorque);
            return joint_torque_command_;
        default:
            throw std::runtime_error(utils::Formatter() << "Unsupported resolution strategy");
    }
}

const Eigen::VectorXd& Controller::computeKKTTorques()
{
    throw std::runtime_error(utils::Formatter() << "computeKKTTorques() is not yet implemented");
    return kkt_torques_;
}

const Eigen::VectorXd& Controller::getJointAccelerationCommand()
{
    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
            joint_acceleration_command_ = problems_.front()->getSolution(ControlVariable::JointSpaceAcceleration);
            return joint_acceleration_command_;
        default:
            throw std::runtime_error(utils::Formatter() << "Unsupported resolution strategy");
    }
}

void Controller::activateTasksAndConstraints()
{
    activateTasks();
    activateConstraints();
}

void Controller::deactivateTasksAndConstraints()
{
    deactivateTasks();
    deactivateConstraints();
}

void Controller::deactivateTasks()
{
    for(auto problem : problems_)
    {
        for(auto t : problem->getTasks())
        {
            if(t->getName() == "DynamicsEquation")
                continue;
            t->deactivate();
        }
    }
}

void Controller::deactivateConstraints()
{
    for(auto problem : problems_)
    {
        for(auto c : problem->getConstraints())
        {
            if(c->getName() == "GlobalRegularisation")
                continue;
            c->deactivate();
        }
    }
}

void Controller::activateTasks()
{
    for(auto problem : problems_)
    {
        for(auto t : problem->getTasks())
        {
            if(t->getName() == "DynamicsEquation")
                continue;
            t->activate();
        }
    }
}

void Controller::activateConstraints()
{
    for(auto problem : problems_)
    {
        for(auto c : problem->getConstraints())
        {
            if(c->getName() == "GlobalRegularisation")
                continue;
            c->activate();
        }
    }
}

bool Controller::tasksAndConstraintsDeactivated()
{
    for(auto problem : problems_)
    {
        for(auto t : problem->getTasks())
        {
            if(t->getName() == "DynamicsEquation")
                continue;
            if(t->getState() != common::TaskBase::Deactivated)
                return false;
        }
        for(auto c : problem->getConstraints())
        {
            if(c->getName() == "GlobalRegularisation")
                continue;
            if(c->getState() != common::TaskBase::Deactivated)
                return false;
        }
    }
    return true;
}

std::shared_ptr<task::RegularisationTask<ControlVariable::X> > Controller::globalRegularization(int level)
{
    if(level < problems_.size())
    {
        auto it = problems_.begin();
        std::advance(it, level);
        auto problem = *it;
        for(auto t : problem->getTasks())
        {
            if(t->getName() == "GlobalRegularisation")
            {
                return std::dynamic_pointer_cast<task::RegularisationTask<ControlVariable::X> >(t);
            }
        }
    }
    return 0;
}

void Controller::insertNewProblem()
{
    LOG_INFO << "Inserting new MOOProblem at level " << problems_.size();
    auto problem = std::make_shared<Problem>(robot_,solver_type_);
    problems_.push_back(problem);

    auto dynamics_equation = std::make_shared<constraint::DynamicsEquationConstraint>("DynamicsEquation");
    auto global_regularisation = std::make_shared<task::RegularisationTask<ControlVariable::X> >("GlobalRegularisation");

    global_regularisation->euclidianNorm().setWeight(1E-5);

    addConstraint(dynamics_equation);
    addTask(global_regularisation);

    dynamics_equation->activate();
    global_regularisation->activate();
}

void Controller::updateTasks(double current_time, double dt)
{
    for(auto problem : problems_)
    {
        for(auto t : problem->getTasks())
        {
            // Checking size
            int cv = problem->getSize(t->getControlVariable());
            if(t->cols() != cv)
            {
                throw std::runtime_error(utils::Formatter() << "Size of task " << t->getName()
                            << " (control var " << t->getControlVariable()
                            << " should be " << cv << " but is " << t->cols() << ")");
            }
            t->update(current_time,dt);
        }
    }
}

void Controller::updateConstraints(double current_time, double dt)
{
    for(auto problem : problems_)
    {
        for(auto c : problem->getConstraints())
        {
            // Checking size
            int cv = problem->getSize(c->getControlVariable());
            if(c->cols() != cv)
            {
                throw std::runtime_error(utils::Formatter() << "Size of constraint " << c->getName()
                            << " (control var " << c->getControlVariable()
                            << " should be " << cv << " but is " << c->cols() << ")");
            }
            c->update(current_time,dt);
        }
    }
}
