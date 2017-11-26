#include <orca/optim/OptimisationVector.h>

#include <orca/constraint/Contact.h>
#include <orca/task/WrenchTask.h>
#include <iostream>
#include <array>

using namespace orca::optim;
using namespace orca::constraint;
using namespace orca::task;
using namespace orca::common;

namespace orca{
    namespace optim{
        static OptimVector __optim_vector_instance__;
    }
}

orca::optim::OptimVector& orca::optim::OptimisationVector()
{
    return __optim_vector_instance__;
}

void OptimVector::addInRegister(QPSolver* qp)
{
    MutexLock lock(mutex);
    if (std::find(std::begin(qps_), std::end(qps_), qp) == std::end(qps_))
    {
        LOG_DEBUG << "[OptimisationVector] Adding QPSolver " << qp;
        qps_.push_back(qp);
        LOG_DEBUG << "----------> Resizing QP ";
        qp->resize();
    }
    else
    {
        LOG_WARNING << "[OptimisationVector] QPSolver " << qp << " is already present";
    }
}

void OptimVector::addInRegister(TaskCommon* t)
{
    MutexLock lock(mutex);
    LOG_DEBUG << "[OptimisationVector] Locking TaskCommon " << t << " var " << t->getControlVariable();
    t->mutex.lock();

    if (std::find(std::begin(commons_), std::end(commons_), t) == std::end(commons_))
    {
        LOG_DEBUG << "[OptimisationVector] Adding TaskCommon " << t << " var " << t->getControlVariable();
        commons_.push_back(t);
        
        if(dynamic_cast<GenericTask*>(t))
        {
            LOG_DEBUG << "----------> Adding GenericTask " << t << " var " << t->getControlVariable();
            tasks_.push_back(dynamic_cast<GenericTask*>(t));
        }

        if(dynamic_cast<GenericConstraint*>(t))
        {
            LOG_DEBUG << "----------> Adding GenericConstraint " << t << " var " << t->getControlVariable();
            constraints_.push_back(dynamic_cast<GenericConstraint*>(t));
        }
        
        if(dynamic_cast<Wrench*>(t))
        {
            LOG_DEBUG << "----------> Adding Wrench " << t << " var " << t->getControlVariable();
            wrenches_.push_back(dynamic_cast<Wrench*>(t));
            this->buildControlVariablesMapping(getNrOfDegreesOfFreedom());
            this->resizeTasks();
            this->resizeConstraints();
        }

        LOG_DEBUG << "----------> Resizing QP ";
        for(auto qp : qps_)
        {
            qp->resize();
        }
    }
    else
    {
        LOG_WARNING << "[OptimisationVector] Task " << t << " already exists in register";
    }
    LOG_DEBUG << "[OptimisationVector] Unlocking TaskCommon " << t << " var " << t->getControlVariable();
    t->mutex.unlock();
}


void OptimVector::resizeTasks()
{
    for(auto task : tasks_)
    {
        if(task->getControlVariable() == ControlVariable::X)
        {
            LOG_DEBUG << "----------> Resizing task " << task << " var " << task->getControlVariable();
            task->resize();
        }
    }
}

void OptimVector::resizeConstraints()
{
    for(auto constr : constraints_)
    {
        if(constr->getControlVariable() == ControlVariable::X)
        {
            LOG_DEBUG << "----------> Resizing constraint " << constr << " var " << constr->getControlVariable();
            constr->resize();
        }
    }
}

bool OptimVector::isInRegister(TaskCommon* t)
{
    MutexLock lock(mutex);
    return std::find(std::begin(commons_), std::end(commons_), t) != std::end(commons_);
}

void OptimVector::removeFromRegister(TaskCommon* t)
{
    MutexLock lock(mutex);
    auto elem_it = std::find(std::begin(commons_), std::end(commons_), t);
    if(elem_it != std::end(commons_))
    {
        commons_.erase(elem_it);
        
        if(dynamic_cast<GenericTask*>(t))
        {
            LOG_DEBUG << "[OptimisationVector] Removing GenericTask " << dynamic_cast<GenericTask*>(t) << " var " << t->getControlVariable();

            tasks_.erase( std::find(std::begin(tasks_), std::end(tasks_), dynamic_cast<GenericTask*>(t)) );
        }

        if(dynamic_cast<GenericConstraint*>(t))
        {
            LOG_DEBUG << "[OptimisationVector] Removing GenericConstraint " << dynamic_cast<GenericConstraint*>(t) << " var " << t->getControlVariable();
            constraints_.erase( std::find(std::begin(constraints_), std::end(constraints_), dynamic_cast<GenericConstraint*>(t)) );
        }
        
        if(dynamic_cast<Wrench*>(t))
        {
            LOG_DEBUG << "----------> Removing Wrench " << t << " var " << t->getControlVariable();
            wrenches_.erase( std::find(std::begin(wrenches_), std::end(wrenches_), dynamic_cast<Wrench*>(t)) );
            
            this->buildControlVariablesMapping(getNrOfDegreesOfFreedom());
            this->resizeTasks();
            this->resizeConstraints();
        }
        
        LOG_DEBUG << "----------> Resizing QP ";
        for(auto qp : qps_)
        {
            qp->resize();
        }
        return;
    }
    else
    {
        LOG_ERROR << "[OptimisationVector] Task " << t << " was not inserted";
    }
}

void OptimVector::removeFromRegister(QPSolver* qp)
{
    MutexLock lock(mutex);
    auto elem_it = std::find(std::begin(qps_), std::end(qps_), qp);
    if(elem_it != std::end(qps_))
    {
        LOG_DEBUG << "[OptimisationVector] Removing QP " << qp;
        qps_.erase(elem_it);
    }
    else
    {
        LOG_ERROR << "[OptimisationVector] QP was not inserted" << qp;
    }
}

void OptimVector::buildControlVariablesMapping(int ndof)
{
    MutexLock lock(mutex);
    ndof_ = ndof;
    is_floating_base_ = true;
    nwrenches_ = wrenches_.size();
    const int fulldim = ndof + (is_floating_base_ ? 6 : 0);
    
    size_map_[    ControlVariable::X                          ] = 2 * fulldim + nwrenches_ * 6;
    size_map_[    ControlVariable::GeneralisedAcceleration    ] = fulldim;
    size_map_[    ControlVariable::FloatingBaseAcceleration   ] = (is_floating_base_ ? 6 : 0);
    size_map_[    ControlVariable::JointSpaceAcceleration     ] = ndof;
    size_map_[    ControlVariable::GeneralisedTorque          ] = fulldim;
    size_map_[    ControlVariable::FloatingBaseWrench         ] = (is_floating_base_ ? 6 : 0);
    size_map_[    ControlVariable::JointSpaceTorque           ] = ndof;
    size_map_[    ControlVariable::ExternalWrench             ] = 6;
    size_map_[    ControlVariable::ExternalWrenches           ] = nwrenches_ * 6;
    size_map_[    ControlVariable::Composite                  ] = 0;
    size_map_[    ControlVariable::None                       ] = 0;
    
    idx_map_[    ControlVariable::X                          ] = 0;
    idx_map_[    ControlVariable::GeneralisedAcceleration    ] = 0;
    idx_map_[    ControlVariable::FloatingBaseAcceleration   ] = 0;
    idx_map_[    ControlVariable::JointSpaceAcceleration     ] = (is_floating_base_ ? 6 : 0);
    idx_map_[    ControlVariable::GeneralisedTorque          ] = fulldim;
    idx_map_[    ControlVariable::FloatingBaseWrench         ] = fulldim;
    idx_map_[    ControlVariable::JointSpaceTorque           ] = fulldim + (is_floating_base_ ? 6 : 0);
    idx_map_[    ControlVariable::ExternalWrench             ] = 2 * fulldim;
    idx_map_[    ControlVariable::ExternalWrenches           ] = 2 * fulldim;
    idx_map_[    ControlVariable::Composite                  ] = 0;
    idx_map_[    ControlVariable::None                       ] = 0;

}

void OptimVector::print() const
{
    MutexLock lock(mutex);
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
        std::cout << "  - " << v << " index " << idx_map_.at(v) << " size " << size_map_.at(v) << '\n';
    }
}

int OptimVector::getNrOfWrenches() const
{
    MutexLock lock(mutex);
    return wrenches_.size();
}

const std::list<Wrench*>& OptimVector::getWrenches() const
{
    MutexLock lock(mutex);
    return wrenches_;
}

const std::list<GenericTask *>& OptimVector::getTasks() const
{
    MutexLock lock(mutex);
    return tasks_;
}

const std::list<TaskCommon *>& OptimVector::getAllCommons() const
{
    MutexLock lock(mutex);
    return commons_;
}

const std::list<GenericConstraint *>& OptimVector::getConstraints() const
{
    MutexLock lock(mutex);
    return constraints_;
}

const std::map<ControlVariable, unsigned int >& OptimVector::getIndexMap() const
{
    MutexLock lock(mutex);
    return idx_map_;
}
const std::map<ControlVariable, unsigned int >& OptimVector::getSizeMap() const
{
    MutexLock lock(mutex);
    return size_map_;
}


int OptimVector::getIndex(ControlVariable var) const
{
    MutexLock lock(mutex);
    // if(ndof_ == 0)
    //     throw std::runtime_error("OptimVector is not initialized");
    return idx_map_.at(var);
}

int OptimVector::getSize(ControlVariable var) const
{
    MutexLock lock(mutex);
    // if(ndof_ == 0)
    //     throw std::runtime_error("OptimVector is not initialized");
    return size_map_.at(var);
}

int OptimVector::getTotalSize() const
{
    return getSize( ControlVariable::X );
}

bool OptimVector::hasFloatingBaseVars() const
{
    MutexLock lock(mutex);
    // if(ndof_ == 0)
    //     throw std::runtime_error("OptimVector is not initialized");
    return is_floating_base_;
}

int OptimVector::getNrOfDegreesOfFreedom() const
{
    MutexLock lock(mutex);
    // if(ndof_ == 0)
    //     throw std::runtime_error("OptimVector is not initialized");
    return ndof_;
}

int OptimVector::configurationSpaceDimension() const
{
    MutexLock lock(mutex);
    // if(ndof_ == 0)
    //     throw std::runtime_error("OptimVector is not initialized");
    return ndof_ + (is_floating_base_ ? 6 : 0);
}

