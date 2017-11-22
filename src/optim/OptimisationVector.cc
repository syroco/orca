#include <orca/optim/OptimisationVector.h>

#include <orca/constraint/Contact.h>
#include <orca/task/WrenchTask.h>
#include <iostream>

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
        LOG_DEBUG << "[OptimisationVector] QPSolver " << qp << " is already present";
    }
}

void OptimVector::addInRegister(TaskCommon* t)
{
    MutexLock lock(mutex);

    if (std::find(std::begin(all_tasks_), std::end(all_tasks_), t) == std::end(all_tasks_))
    {
        LOG_DEBUG << "[OptimisationVector] Adding TaskCommon " << t << " var " << t->getControlVariable();
        all_tasks_.push_back(t);
        
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

        for(auto qp : qps_)
        {
            qp->resize();
        }
    }
    LOG_ERROR << "[OptimisationVector] Task " << t << " already exists in register";
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

void OptimVector::removeFromRegister(TaskCommon* t)
{
    MutexLock lock(mutex);
    auto elem_it = std::find(std::begin(all_tasks_), std::end(all_tasks_), t);
    if(elem_it != std::end(all_tasks_))
    {
        all_tasks_.erase(elem_it);
        
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
        
        for(auto qp : qps_)
        {
            qp->resize();
        }
        return;
    }
    LOG_ERROR << "[OptimisationVector] Task " << t << " was not inserted";
}

void OptimVector::removeFromRegister(QPSolver* qp)
{
    MutexLock lock(mutex);
    auto elem_it = std::find(std::begin(qps_), std::end(qps_), qp);
    if(elem_it != std::end(qps_))
    {
        LOG_DEBUG << "[OptimisationVector] Removing QP " << qp;
        qps_.erase(elem_it);
        return;
    }
    LOG_ERROR << "[OptimisationVector] QP was not inserted" << qp;
    return;
}

void OptimVector::buildControlVariablesMapping(int ndof)
{
    MutexLock lock(mutex);
    ndof_ = ndof;
    is_floating_base_ = true;
    nwrenches_ = getNrOfWrenches();
    const int fulldim = ndof + (is_floating_base_ ? 6 : 0);
    
    size_map_[    ControlVariable::X                          ] = 2 * fulldim + nwrenches_ * 6;
    size_map_[    ControlVariable::GeneralisedAcceleration    ] = fulldim;
    size_map_[    ControlVariable::FloatingBaseAcceleration   ] = (is_floating_base_ ? 6 : 0);
    size_map_[    ControlVariable::JointSpaceAcceleration     ] = ndof;
    size_map_[    ControlVariable::GeneralisedTorque          ] = fulldim;
    size_map_[    ControlVariable::FloatingBaseWrench         ] = (is_floating_base_ ? 6 : 0);
    size_map_[    ControlVariable::JointSpaceTorque           ] = ndof;
    size_map_[    ControlVariable::ExternalWrench             ] = 6;
    
    idx_map_[    ControlVariable::X                          ] = 0;
    idx_map_[    ControlVariable::GeneralisedAcceleration    ] = 0;
    idx_map_[    ControlVariable::FloatingBaseAcceleration   ] = 0;
    idx_map_[    ControlVariable::JointSpaceAcceleration     ] = (is_floating_base_ ? 6 : 0);
    idx_map_[    ControlVariable::GeneralisedTorque          ] = fulldim;
    idx_map_[    ControlVariable::FloatingBaseWrench         ] = fulldim;
    idx_map_[    ControlVariable::JointSpaceTorque           ] = fulldim + (is_floating_base_ ? 6 : 0);
    idx_map_[    ControlVariable::ExternalWrench             ] = 2 * fulldim;

}

void OptimVector::print() const
{
    MutexLock lock(mutex);
//     LOG_DEBUG << "Optimisation OptimVector X Total size : "<< getSize(ControlVariable::X) << std::endl;
//     LOG_DEBUG << "  Acceleration index " << getIndex(ControlVariable::Acceleration) << " size " << getSize(ControlVariable::Acceleration) << std::endl;
//     LOG_DEBUG << "  Torque       index " << getIndex(ControlVariable::Torque) << " size " << getSize(ControlVariable::Torque) << std::endl;
//     LOG_DEBUG << "  Wrench       index " << getIndex(ControlVariable::ExternalWrench) << " size " << getSize(ControlVariable::ExternalWrench)
//                         << " * NWrenches (" << getNrOfWrenches() <<")"
//                         << std::endl;
}

int OptimVector::getNrOfWrenches() const
{
    MutexLock lock(mutex);
    return wrenches_.size();
}

const std::list<Wrench*> OptimVector::getWrenches() const
{
    MutexLock lock(mutex);
    return wrenches_;
}

const std::list<GenericTask *> OptimVector::getTasks() const
{
    MutexLock lock(mutex);
    return tasks_;
}

const std::list<GenericConstraint *> OptimVector::getConstraints() const
{
    MutexLock lock(mutex);
    return constraints_;
}

int OptimVector::getIndex(ControlVariable var) const
{
    MutexLock lock(mutex);
    if(ndof_ == 0)
        throw std::runtime_error("OptimVector is not initialized");
    return idx_map_.at(var);
}

int OptimVector::getSize(ControlVariable var) const
{
    MutexLock lock(mutex);
    if(ndof_ == 0)
        throw std::runtime_error("OptimVector is not initialized");
    return size_map_.at(var);
}

int OptimVector::getTotalSize() const
{
    return getSize( ControlVariable::X );
}

bool OptimVector::hasFloatingBaseVars() const
{
    MutexLock lock(mutex);
    if(ndof_ == 0)
        throw std::runtime_error("OptimVector is not initialized");
    return is_floating_base_;
}

int OptimVector::getNrOfDegreesOfFreedom() const
{
    MutexLock lock(mutex);
    if(ndof_ == 0)
        throw std::runtime_error("OptimVector is not initialized");
    return ndof_;
}

int OptimVector::configurationSpaceDimension() const
{
    MutexLock lock(mutex);
    if(ndof_ == 0)
        throw std::runtime_error("OptimVector is not initialized");
    return ndof_ + (is_floating_base_ ? 6 : 0);
}

