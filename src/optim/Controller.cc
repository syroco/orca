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

bool Controller::update(double current_time, double dt)
{
    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
            updateTasks(current_time,dt);
            updateConstraints(current_time,dt);
            problems_.front()->build();
            return problems_.front()->solve();
        default:
            throw std::runtime_error(utils::Formatter() << "unsupported resolution strategy");
    }
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

Eigen::VectorXd Controller::getJointTorqueCommand()
{
    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
            return problems_.front()->getSolution(ControlVariable::JointSpaceTorque);
        default:
            throw std::runtime_error(utils::Formatter() << "Unsupported resolution strategy");
    }
}

Eigen::VectorXd Controller::getJointAccelerationCommand()
{
    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
            return problems_.front()->getSolution(ControlVariable::JointSpaceAcceleration);
        default:
            throw std::runtime_error(utils::Formatter() << "Unsupported resolution strategy");
    }
}

void Controller::activateAll(double current_time)
{
    for(auto problem : problems_)
    {
        for(auto t : problem->getTasks())
        {
            t->activate();
        }
        for(auto c : problem->getConstraints())
        {
            c->activate();
        }
    }
}

void Controller::deactivateAll(double current_time)
{
    for(auto problem : problems_)
    {
        for(auto t : problem->getTasks())
        {
            t->deactivate();
        }
        for(auto c : problem->getConstraints())
        {
            c->deactivate();
        }
    }
}

bool Controller::allDeactivated()
{
    for(auto problem : problems_)
    {
        for(auto t : problem->getTasks())
        {
            if(t->getState() != common::TaskBase::Deactivated)
                return false;
        }
        for(auto c : problem->getConstraints())
        {
            if(c->getState() != common::TaskBase::Deactivated)
                return false;
        }
    }
    return true;
}

void Controller::insertNewProblem()
{
    LOG_INFO << "Inserting new MOOProblem at level " << problems_.size();
    auto problem = std::make_shared<Problem>(robot_,solver_type_);
    problem->qpSolver()->setPrintLevel(0);

    auto dynamics_equation = std::make_shared<constraint::DynamicsEquationConstraint>("DynamicsEquation");
    auto global_regularisation = std::make_shared<task::RegularisationTask<ControlVariable::X> >("GlobalRegularisation");

    dynamics_equation->setRobotModel(robot_);
    dynamics_equation->setProblem(problem);

    global_regularisation->setRobotModel(robot_);
    global_regularisation->setProblem(problem);

    global_regularisation->euclidianNorm().setWeight(1E-5);

    problem->addConstraint(dynamics_equation);
    problem->addTask(global_regularisation);

    problems_.push_back(problem);
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
