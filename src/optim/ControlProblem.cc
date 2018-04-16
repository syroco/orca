#include "orca/optim/ControlProblem.h"
#include "orca/common/Wrench.h"
#include "orca/common/TaskBase.h"
#include "orca/task/GenericTask.h"
#include "orca/constraint/GenericConstraint.h"
#include "orca/robot/RobotDynTree.h"

using namespace orca::common;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::robot;

bool Problem::remove(TaskBase* task_base)
{
    if(isTask(task_base))
    {
        if(taskExists(task_base))
        {
            LOG_INFO << "Removing task " << task_base->getName();
            return true;
        }
        else
        {
            LOG_INFO << "Task " << task_base->getName() << " was not inserted, not removing";
            return false;
        }
    }
    
    if(isConstraint(task_base) && constraintExists(task_base))
    {
        LOG_INFO << "Removing constraint " << task_base->getName();
        return true;
    }
    if(isWrench(task_base) && constraintExists(task_base))
    {
        LOG_INFO << "Removing wrench " << task_base->getName();
        return true;
    }
    LOG_ERROR << "Task base " << task_base->getName() << " not found in problem";
    return false;
}

bool Problem::add(TaskBase* task_base)
{
    if(isTask(task_base) && !taskExists(task_base))
    {
        LOG_INFO << "Adding task " << task_base->getName();
        tasks_.push_back(dynamic_cast<GenericTask*>(task_base));
        return true;
    }
    if(isConstraint(task_base) && !constraintExists(task_base))
    {
        LOG_INFO << "Adding constraint " << task_base->getName();
        constraints_.push_back(dynamic_cast<GenericConstraint*>(task_base));
        return true;
    }
    if(isWrench(task_base) && !constraintExists(task_base))
    {
        LOG_INFO << "Adding wrench " << task_base->getName();
        wrenches_.push_back(dynamic_cast<Wrench*>(task_base));
        buildControlVariablesMapping(); // size has changed, so we need to resize X dependent TaskBases and the solver
        resizeTasks();
        resizeConstraints();
        return true;
    }
    LOG_ERROR << "Task base " << task_base->getName() << " is already in problem";
    return false;
}


bool Problem::isTask(const TaskBase* task_base)
{
    return dynamic_cast<const GenericTask*>(task_base);
}

bool Problem::isConstraint(const TaskBase* task_base)
{
    return dynamic_cast<const GenericConstraint*>(task_base);
}

bool Problem::isWrench(const TaskBase* task_base)
{
    return dynamic_cast<const Wrench*>(task_base);
}

bool Problem::taskExists(const TaskBase* task_base)
{
    return std::find(tasks_.begin(),tasks_.end(),task_base) != tasks_.end();
}

bool Problem::constraintExists(const TaskBase* task_base)
{
    return std::find(constraints_.begin(),constraints_.end(),task_base) != constraints_.end();
}

bool Problem::wrenchExists(const TaskBase* task_base)
{
    return std::find(wrenches_.begin(),wrenches_.end(),task_base) != wrenches_.end();
}

void Problem::setRobotModel(std::shared_ptr<RobotDynTree> robot)
{
    this->robot_ = robot;
    this->ndof_ = robot->getNrOfDegreesOfFreedom();
    this->configuration_space_dim_ = this->ndof_ + 6;
    buildControlVariablesMapping();
}

void Problem::setQPSolver(QPSolver::SolverType qpsolver_type)
{
    this->qpsolver_ = std::make_shared<QPSolver>(qpsolver_type);
}

void Problem::print() const
{
    std::cout << "Problem objects : " << std::endl;
    std::cout << "      Tasks" << std::endl;
    for(auto t : this->tasks_)
    {
        std::cout << "          " << t->getName() << std::endl;
    }
    std::cout << "      Constraints" << std::endl;
    for(auto c : this->constraints_)
    {
        std::cout << "          " << c->getName() << std::endl;
    }
    std::cout << "Optimisation Vector : " << '\n';
    const auto all_variables =
    {
          ControlVariable::X
        , ControlVariable::GeneralisedAcceleration
        , ControlVariable::FloatingBaseAcceleration
        , ControlVariable::JointSpaceAcceleration
        , ControlVariable::GeneralisedTorque
        , ControlVariable::FloatingBaseWrench
        , ControlVariable::JointSpaceTorque
        , ControlVariable::ExternalWrench
        , ControlVariable::ExternalWrenches
        , ControlVariable::Composite
        , ControlVariable::None
    };
    for(auto v : all_variables)
    {
        std::cout << "  - " << v << " index " << getIndex(v) << " size " << getSize(v) << '\n';
    }
}

const std::list< orca::common::Wrench* >& Problem::getWrenches() const
{
    return wrenches_;
}

const std::list< GenericTask* >& Problem::getTasks() const
{
    return tasks_;
}

const std::list< GenericConstraint* >& Problem::getConstraints() const
{
    return constraints_;
}

int Problem::getConfigurationSpaceDimension() const
{
    return configuration_space_dim_;
}

const std::map<ControlVariable, unsigned int >& Problem::getIndexMap() const
{
    return index_map_;
}

const std::map<ControlVariable, unsigned int >& Problem::getSizeMap() const
{
    return size_map_;
}

int Problem::getIndex(ControlVariable var) const
{
    return index_map_.at(var);
}

int Problem::getSize(ControlVariable var) const
{
    return size_map_.at(var);
}

int Problem::getTotalSize() const
{
    return getSize( ControlVariable::X );
}

void Problem::buildControlVariablesMapping()
{
    nwrenches_ = wrenches_.size();
    const int fulldim = getConfigurationSpaceDimension();

    size_map_[    ControlVariable::X                          ] = 2 * fulldim + nwrenches_ * 6;
    size_map_[    ControlVariable::GeneralisedAcceleration    ] = fulldim;
    size_map_[    ControlVariable::FloatingBaseAcceleration   ] = (is_floating_base_ ? 6 : 0);
    size_map_[    ControlVariable::JointSpaceAcceleration     ] = ndof_;
    size_map_[    ControlVariable::GeneralisedTorque          ] = fulldim;
    size_map_[    ControlVariable::FloatingBaseWrench         ] = (is_floating_base_ ? 6 : 0);
    size_map_[    ControlVariable::JointSpaceTorque           ] = ndof_;
    size_map_[    ControlVariable::ExternalWrench             ] = 6;
    size_map_[    ControlVariable::ExternalWrenches           ] = nwrenches_ * 6;
    size_map_[    ControlVariable::Composite                  ] = 0;
    size_map_[    ControlVariable::None                       ] = 0;

    index_map_[    ControlVariable::X                          ] = 0;
    index_map_[    ControlVariable::GeneralisedAcceleration    ] = 0;
    index_map_[    ControlVariable::FloatingBaseAcceleration   ] = 0;
    index_map_[    ControlVariable::JointSpaceAcceleration     ] = (is_floating_base_ ? 6 : 0);
    index_map_[    ControlVariable::GeneralisedTorque          ] = fulldim;
    index_map_[    ControlVariable::FloatingBaseWrench         ] = fulldim;
    index_map_[    ControlVariable::JointSpaceTorque           ] = fulldim + (is_floating_base_ ? 6 : 0);
    index_map_[    ControlVariable::ExternalWrench             ] = 2 * fulldim;
    index_map_[    ControlVariable::ExternalWrenches           ] = 2 * fulldim;
    index_map_[    ControlVariable::Composite                  ] = 0;
    index_map_[    ControlVariable::None                       ] = 0;

}

void Problem::resizeEverybody()
{
    buildControlVariablesMapping();
    resizeTasks();
    resizeConstraints();
    resizeSolver();
}

void Problem::resizeSolver()
{

}


void Problem::resizeTasks()
{
    for(auto task : tasks_)
    {
        if(task->getControlVariable() == ControlVariable::X)
        {
            LOG_DEBUG << "Resizing task " << task;
            task->resize();
        }
    }
}

void Problem::resizeConstraints()
{
    for(auto constr : constraints_)
    {
        if(constr->getControlVariable() == ControlVariable::X)
        {
            LOG_DEBUG << "Resizing constraint " << constr;
            constr->resize();
        }
    }
}