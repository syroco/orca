#include "orca/optim/Controller.h"
#include "orca/common/Wrench.h"
#include "orca/common/TaskBase.h"
#include "orca/constraint/Contact.h"
#include "orca/task/RegularisationTask.h"
#include "orca/constraint/DynamicsEquationConstraint.h"

using namespace orca;
using namespace orca::optim;
using namespace orca::utils;

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
        orca_throw(Formatter() << "Only ResolutionStrategy::OneLevelWeighted is supported for now");
    }
    insertNewLevel();
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
        orca_throw(Formatter() << "Robot is not set");
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

bool Controller::isProblemDry(std::shared_ptr<const optim::Problem> problem)
{
    // Returns true if there is only one task computing something,
    // and it's the GlobalRegularisation
    int ntasks_computing = 0;
    int task_index = 0;
    for(auto t : problem->getTasks())
    {
        if(t->isComputing())
            ntasks_computing++;

        if(ntasks_computing > 1)
            return false;

        task_index++;
    }
    // Here ntasks_computing == 1
    for(auto t : problem->getTasks())
    {
        if(t->isComputing() || t->getName() == "GlobalRegularisation")
            return true;
    }
    return false;
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
            auto problem = getProblemAtLevel(0);
            problem->build();
            solution_found_ = problem->solve();

            if(this->update_cb_)
                this->update_cb_(current_time,dt);

            static bool print_warning = true;
            if(solution_found_ && isProblemDry(problem) && print_warning)
            {
                print_warning = false;
                LOG_WARNING << "\n\n"
                    <<" Solution found but the problem is dry !\n"
                    << "It means that an optimal solution is found but the problem \n"
                    << "only has one task computing anything, ans it's the"
                    << "GlobalRegularisation task (This will only be printed once)\n\n"
                    << "/!\\ Resulting torques will cause the robot to fall /!\\";
            }

            return solution_found_;
        }
        default:
            orca_throw(Formatter() << "unsupported resolution strategy");
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
            orca_throw(Formatter() << "unsupported resolution strategy");
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

std::shared_ptr<task::GenericTask> Controller::getTask(const std::string& name, int level)
{
    auto problem = getProblemAtLevel(level);
    for(auto t : problem->getTasks())
        if(t->getName() == name)
            return t;
    orca_throw(Formatter() << "Task " << name << " does not exist at level " << level);
}

std::shared_ptr<task::GenericTask> Controller::getTask(unsigned int index, int level)
{
    auto problem = getProblemAtLevel(level);
    if(problem->getTasks().size() < index)
        orca_throw(Formatter() <<"Problem at level " << level << " only have " << problem->getTasks().size() << "tasks, cannot retrieve task index " << index);

    auto it = problem->getTasks().begin();
    std::advance(it, level);
    return *it;
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
            orca_throw(Formatter() << "Unsupported resolution strategy");
    }
}

const Eigen::VectorXd& Controller::getJointTorqueCommand(bool remove_gravity_torques /*= false*/
                                                    , bool remove_coriolis_torques /*= false*/)
{
    switch (resolution_strategy_)
    {
        case ResolutionStrategy::OneLevelWeighted:
            if(!solutionFound())
                orca_throw(Formatter() << "Cannot return JointTorqueCommand as the problem is not solved");

            joint_torque_command_ = problems_.front()->getSolution(ControlVariable::JointSpaceTorque);

            if(remove_gravity_torques || remove_gravity_torques_)
                joint_torque_command_ -= robot_->getJointGravityTorques();

            if(remove_coriolis_torques || remove_coriolis_torques_)
                joint_torque_command_ -= robot_->getJointCoriolisTorques();

            return joint_torque_command_;

        default:
            orca_throw(Formatter() << "Unsupported resolution strategy");
    }
}

const Eigen::VectorXd& Controller::computeKKTTorques()
{
    orca_throw(Formatter() << "computeKKTTorques() is not yet implemented");
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
            orca_throw(Formatter() << "Unsupported resolution strategy");
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
            if(t->getName() == "GlobalRegularisation")
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
            if(c->getName() == "DynamicsEquation")
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
            if(t->getName() == "GlobalRegularisation")
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
            if(c->getName() == "DynamicsEquation")
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
            if(t->getName() == "GlobalRegularisation")
                continue;
            if(t->getState() != common::TaskBase::Deactivated)
                return false;
        }
        for(auto c : problem->getConstraints())
        {
            if(c->getName() == "DynamicsEquation")
                continue;
            if(c->getState() != common::TaskBase::Deactivated)
                return false;
        }
    }
    return true;
}

void Controller::removeGravityTorquesFromSolution(bool do_remove)
{
    remove_gravity_torques_ = do_remove;
}
void Controller::removeCoriolisTorquesFromSolution(bool do_remove)
{
    remove_coriolis_torques_ = do_remove;
}

std::shared_ptr<Problem> Controller::getProblemAtLevel(int level)
{
    if(level < problems_.size())
    {
        auto it = problems_.begin();
        std::advance(it, level);
        return *it;
    }
    orca_throw(Formatter() << "Level " << level << " does not exist.\n"
                            << "There is only " << problems_.size() << " level(s)");
}

std::shared_ptr<task::RegularisationTask<ControlVariable::X> > Controller::globalRegularization(int level)
{
    auto problem = getProblemAtLevel(level);
    for(auto t : problem->getTasks())
    {
        if(t->getName() == "GlobalRegularisation")
        {
            return std::dynamic_pointer_cast<task::RegularisationTask<ControlVariable::X> >(t);
        }
    }
    return 0;
}

void Controller::insertNewLevel()
{
    LOG_INFO << "Inserting new dry Problem at level " << problems_.size();
    auto problem = std::make_shared<Problem>(robot_,solver_type_);
    problems_.push_back(problem);

    auto dynamics_equation = std::make_shared<constraint::DynamicsEquationConstraint>("DynamicsEquation");
    auto global_regularisation = std::make_shared<task::RegularisationTask<ControlVariable::X> >("GlobalRegularisation");

    global_regularisation->euclidianNorm().setWeight(1E-8);

    addConstraint(dynamics_equation);
    addTask(global_regularisation);

    LOG_INFO << "Controller has now " << problems_.size() << " levels";
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
                orca_throw(Formatter() << "Size of task " << t->getName()
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
                orca_throw(Formatter() << "Size of constraint " << c->getName()
                            << " (control var " << c->getControlVariable()
                            << " should be " << cv << " but is " << c->cols() << ")");
            }
            c->update(current_time,dt);
        }
    }
}
