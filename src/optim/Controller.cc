#include "orca/optim/Controller.h"
#include "orca/common/Wrench.h"
#include "orca/common/TaskBase.h"
#include "orca/constraint/Contact.h"
#include "orca/task/RegularisationTask.h"
#include "orca/constraint/DynamicsEquationConstraint.h"
#include "orca/common/ParameterSharedPtr.h"

using namespace orca;
using namespace orca::optim;
using namespace orca::utils;
using namespace orca::common;
using namespace orca::robot;

Controller::Controller(const std::string& name)
: ConfigurableOrcaObject(name)
{
    this->addParameter("tasks",&tasks_);
    this->addParameter("constraints",&constraints_);
    this->addParameter("robot_model",&robot_);
    this->addParameter("resolution_strategy",&resolution_strategy_);
    this->addParameter("qpsolver_implementation",&solver_type_);
    this->addParameter("remove_gravity_torques_from_solution",&remove_gravity_torques_, ParamPolicy::Optional);
    this->addParameter("remove_coriolis_torques_from_solution",&remove_coriolis_torques_, ParamPolicy::Optional);
    this->onConfigureSuccess([&](){

        LOG_WARNING << "[" << getName() << "]" << " Config sucessfully loaded ";
        
        joint_acceleration_command_.setZero(robot()->getNrOfDegreesOfFreedom());
        joint_torque_command_.setZero(robot()->getNrOfDegreesOfFreedom());
        kkt_torques_.setZero(robot()->getNrOfDegreesOfFreedom());

        insertNewLevel();
        
        LOG_WARNING << "[" << getName() << "]" << " Adding " << tasks_.get().size() << " tasks "
            << " and " << constraints_.get().size() << " constraints";
        
        for(auto task : tasks_.get())
            this->addTask(task);
        for(auto constraint: constraints_.get())
            this->addConstraint(constraint);
    });
}

Controller::Controller(const std::string& name
    , std::shared_ptr<robot::RobotModel> robot
    ,ResolutionStrategy resolution_strategy
    ,QPSolverImplType solver_type)
: Controller(name)
{
    resolution_strategy_ = resolution_strategy;
    solver_type_ = solver_type;
    robot_ = robot;
    
    joint_acceleration_command_.setZero(robot->getNrOfDegreesOfFreedom());
    joint_torque_command_.setZero(robot->getNrOfDegreesOfFreedom());
    kkt_torques_.setZero(robot->getNrOfDegreesOfFreedom());

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

std::shared_ptr<robot::RobotModel> Controller::robot()
{
    if(!robot_.get())
        orca_throw(Formatter() << "Robot is not set");
    return robot_.get();
}

void Controller::setRobotModel(std::shared_ptr<robot::RobotModel> robot)
{
    robot_ = robot;
    for(auto problem : problems_)
        problem->setRobotModel(robot);
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
    MutexLock lock(mutex);
    solution_found_ = false;

    switch (getResolutionStrategy())
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

            return solution_found_;
        }
        default:
            orca_throw(Formatter() << "unsupported resolution strategy");
    }
    return false;
}

bool Controller::solutionFound() const
{
    return solution_found_;
}

ResolutionStrategy Controller::getResolutionStrategy() const
{
    return resolution_strategy_.get();
}

common::ReturnCode Controller::getReturnCode() const
{
    switch (getResolutionStrategy())
    {
        case ResolutionStrategy::OneLevelWeighted:
            return problems_.front()->getReturnCode();
        default:
            orca_throw(Formatter() << "unsupported resolution strategy");
    }
    return common::ReturnCode::RET_ERROR_UNDEFINED;
}

bool Controller::addTaskFromString(const std::string& task_description)
{
    try
    {
        Parameter<task::GenericTask::Ptr> param;
        param.loadFromString(task_description);
        return this->addTask(param.get());
    } 
    catch(std::exception& e)
    {
        LOG_WARNING << "Could not add task from string : " << e.what();
    }
    return false;
}

bool Controller::addConstraintFromString(const std::string& cstr_description)
{
    try
    {
        Parameter<constraint::GenericConstraint::Ptr> param;
        if(!param.loadFromString(cstr_description))
            return false;
        return this->addConstraint(param.get());
    } 
    catch(std::exception& e)
    {
        LOG_WARNING << "Could not add constraint from string : " << e.what();
    }
    return false;
}

bool Controller::addTask(std::shared_ptr<task::GenericTask> task)
{
    if(!task)
    {
        return false;
    }
    if(getResolutionStrategy() == ResolutionStrategy::OneLevelWeighted)
    {
        if(!problems_.front()->taskExists(task))
        {
            task->setRobotModel(robot());
            if(task->dependsOnProblem())
                task->setProblem(problems_.front());
            return problems_.front()->addTask(task);
        }
        else
        {
            LOG_WARNING << "Task " << task->getName() << " already exists.";
            return true;
        }
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
    return nullptr;
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
    if(getResolutionStrategy() == ResolutionStrategy::OneLevelWeighted)
    {
        if(!problems_.front()->constraintExists(cstr))
        {
            cstr->setRobotModel(robot_.get());
            if(cstr->dependsOnProblem())
                cstr->setProblem(problems_.front());
            return problems_.front()->addConstraint(cstr);
        }
        else
        {
            LOG_WARNING << "Constraint " << cstr->getName() << " already exists.";
            return true;
        }

    }
    return false;
}

const Eigen::VectorXd& Controller::getSolution()
{
    switch (getResolutionStrategy())
    {
        case ResolutionStrategy::OneLevelWeighted:
            return problems_.front()->getSolution();
        default:
            orca_throw(Formatter() << "Unsupported resolution strategy");
    }
    // to fix warnings
    return __fix_warnings__;
}

const Eigen::VectorXd& Controller::getJointTorqueCommand(bool remove_gravity_torques /*= false*/
                                                    , bool remove_coriolis_torques /*= false*/)
{
    switch (getResolutionStrategy())
    {
        case ResolutionStrategy::OneLevelWeighted:
        {
            auto problem = problems_.front();

            static bool print_warning = true;
            if(solution_found_ && isProblemDry(problem) && print_warning)
            {
                print_warning = false;
                LOG_WARNING << "\n\n"
                    <<" Solution found but the problem is dry !\n"
                    << "It means that an optimal solution is found but the problem \n"
                    << "only has one task computing, and it's the "
                    << "GlobalRegularisation task (This will only be printed once)\n\n"
                    << "/!\\ Resulting torques might cause the robot to fall /!\\";
            }

            if(!solutionFound())
                orca_throw(Formatter() << "Cannot return JointTorqueCommand as the problem is not solved."
                    <<"Use controller.solutionFound() to check if the controller computed a valid solution");

            joint_torque_command_ = problem->getSolution(ControlVariable::JointTorque);

            if(remove_gravity_torques || remove_gravity_torques_.get())
                joint_torque_command_ -= robot()->getJointGravityTorques();

            if(remove_coriolis_torques || remove_coriolis_torques_.get())
                joint_torque_command_ -= robot()->getJointCoriolisTorques();

            return joint_torque_command_;
        }
        default:
            orca_throw(Formatter() << "Unsupported resolution strategy");
    }
    // to fix warnings
    return joint_torque_command_;
}

const Eigen::VectorXd& Controller::computeKKTTorques()
{
    orca_throw(Formatter() << "computeKKTTorques() is not yet implemented");
    return kkt_torques_;
}

const Eigen::VectorXd& Controller::getJointAccelerationCommand()
{
    switch (getResolutionStrategy())
    {
        case ResolutionStrategy::OneLevelWeighted:
            joint_acceleration_command_ = problems_.front()->getSolution(ControlVariable::JointAcceleration);
            return joint_acceleration_command_;
        default:
            orca_throw(Formatter() << "Unsupported resolution strategy");
    }
    // To fix warnings
    return joint_acceleration_command_;
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
    remove_gravity_torques_.get() = do_remove;
}
void Controller::removeCoriolisTorquesFromSolution(bool do_remove)
{
    remove_coriolis_torques_.get() = do_remove;
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
    return nullptr;
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
    auto problem = std::make_shared<Problem>();
    problem->setRobotModel(robot());
    problem->setImplementationType(solver_type_.get());
    problems_.push_back(problem);

    auto dynamics_equation = std::make_shared<constraint::DynamicsEquationConstraint>("DynamicsEquation");
    auto global_regularisation = std::make_shared<task::RegularisationTask<ControlVariable::X> >("GlobalRegularisation");
    
    // NOTE: The order matters : 1rst things first to be updated
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
