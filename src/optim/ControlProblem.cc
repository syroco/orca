#include "orca/optim/ControlProblem.h"
#include "orca/common/Wrench.h"
#include "orca/task/GenericTask.h"
#include "orca/constraint/GenericConstraint.h"
#include "orca/robot/RobotDynTree.h"

using namespace orca::common;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::robot;

bool MultiObjectiveOptimisationProblem::addTask(std::shared_ptr<GenericTask> task)
{
    if(taskExists(task))
    {
        LOG_WARNING << "Task " << task->getName() << " already exists";
        return false;
    }
    this->tasks_.push_back(task);
    return taskExists(task);
}

bool MultiObjectiveOptimisationProblem::taskExists(std::shared_ptr<GenericTask> task)
{
    for(auto t : this->tasks_)
        if(t.get() == task.get())
            return true;
    return false;
}

bool MultiObjectiveOptimisationProblem::constraintExists(std::shared_ptr<GenericConstraint> cstr)
{
    for(auto c : this->constraints_)
        if(c.get() == cstr.get())
            return true;
    return false;
}

void MultiObjectiveOptimisationProblem::setRobotModel(std::shared_ptr<RobotDynTree> robot)
{
    this->ndof_ = robot->getNrOfDegreesOfFreedom();
    this->configuration_space_dim_ = this->ndof_ + 6;
}

void MultiObjectiveOptimisationProblem::print()
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
}

const std::list< std::shared_ptr<orca::common::Wrench> >& MultiObjectiveOptimisationProblem::getWrenches() const
{
    return wrenches_;
}

const std::list< std::shared_ptr<GenericTask> >& MultiObjectiveOptimisationProblem::getTasks() const
{
    return tasks_;
}

const std::list< std::shared_ptr<GenericConstraint> >& MultiObjectiveOptimisationProblem::getConstraints() const
{
    return constraints_;
}

int MultiObjectiveOptimisationProblem::getConfigurationSpaceDimension() const
{
    return configuration_space_dim_;
}

const std::map<ControlVariable, unsigned int >& MultiObjectiveOptimisationProblem::getIndexMap() const
{
    return index_map_;
}

const std::map<ControlVariable, unsigned int >& MultiObjectiveOptimisationProblem::getSizeMap() const
{
    return size_map_;
}

int MultiObjectiveOptimisationProblem::getIndex(ControlVariable var) const
{
    return index_map_.at(var);
}

int MultiObjectiveOptimisationProblem::getSize(ControlVariable var) const
{
    return size_map_.at(var);
}

int MultiObjectiveOptimisationProblem::getTotalSize() const
{
    return getSize( ControlVariable::X );
}