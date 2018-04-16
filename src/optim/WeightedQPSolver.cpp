#include "orca/optim/WeightedQPSolver.h"

#include "orca/math/GenericFunction.h"

using namespace orca::optim;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::math;
using namespace orca::common;

void WeightedQPSolver::resize()
{
    LOG_DEBUG << "[WQP] WeightedQPSolver::resize()";
    MutexLock lock(this->mutex);
    
    LOG_DEBUG << "[WQP] Checking if we need resising";
    
    constraints_ =  this->problem()->getConstraints();
    tasks_ = this->problem()->getTasks();
    idx_map_ = this->problem()->getIndexMap();
    size_map_ = this->problem()->getSizeMap();
    
    const int nvars = size_map_[ControlVariable::X];
    int number_of_constraints_rows = 0;

    for(auto constr : constraints_)
    {
        MutexLock lock(constr->mutex);
        
        if(constr->getConstraintMatrix().isIdentity())
        {
            LOG_DEBUG << "[WQP] resize() Detecting lb < x < ub constraint, not adding rows" << constr;
            // Detecting lb < x < ub constraint
        }
        else
        {
            LOG_DEBUG << "[WQP] resize() Adding constraint rows ";
            number_of_constraints_rows += constr->rows();
            LOG_DEBUG << "[WQP] resize() Number of rows is now  " << number_of_constraints_rows ;
        }
    }
    LOG_DEBUG << "[WQP] resize() we are now at  " << data_.H_.rows() << "x" << data_.A_.rows();
    LOG_DEBUG << "[WQP] resize() we ask to resize to  " << nvars << "x" << number_of_constraints_rows;
    QPSolver::resizeInternal(nvars,number_of_constraints_rows);
}

void WeightedQPSolver::buildOptimisationProblem()
{
    MutexLock lock(this->mutex);

    // Reset H and g
    data_.reset();

    int iwrench = 0;
    for(auto task : tasks_)
    {
        MutexLock lock(task->mutex());

        int start_idx = idx_map_[task->getControlVariable()];

        int nrows = task->getQuadraticCost().rows();
        int ncols = task->getQuadraticCost().cols();

        if(task->getControlVariable() == ControlVariable::ExternalWrench)
        {
             start_idx += iwrench * 6;
             iwrench++;
        }


        if(start_idx + nrows <= data_.H_.rows() && start_idx + ncols <= data_.H_.cols())
        {
            if(task->isActivated())
            {
                data_.H_.block(start_idx, start_idx, nrows, ncols).noalias()  += task->getWeight() * task->getQuadraticCost().getHessian();
                data_.g_.segment(start_idx ,ncols).noalias()                  += task->getWeight() * task->getQuadraticCost().getGradient();
            }
        }
        else
        {
            // Error
            task->print();
            throw std::runtime_error(util::Formatter() << "[WQP] Task " << task->getName() << " ptr " << task << " << block of size (" << Size(nrows,ncols) << ")"
                      << "\nCould not fit at index (" << Size(start_idx,start_idx) << ")"
                      << "\nBecause H size is (" << Size(data_.H_) << ")");
        }
    }

    int iAwrench = 0;
    iwrench = 0;
    int row_idx = 0;
    for(auto constr : constraints_)
    {
        MutexLock lock(constr->mutex());
        
        int start_idx = idx_map_[constr->getControlVariable()];

        int nrows = constr->rows();
        int ncols = constr->cols();

        if(constr->getConstraintMatrix().isIdentity())
        {
            if(constr->getControlVariable() == ControlVariable::ExternalWrench)
            {
                start_idx += iwrench * 6;
                iwrench++;
            }

            if(start_idx + nrows <= data_.lb_.size() )
            {
                if(constr->isActivated())
                {
                    data_.lb_.segment(start_idx ,nrows) = data_.lb_.segment(start_idx ,nrows).cwiseMax(constr->getLowerBound());
                    data_.ub_.segment(start_idx ,nrows) = data_.ub_.segment(start_idx ,nrows).cwiseMin(constr->getUpperBound());
                }
            }
            else
            {
                // Error
                throw std::runtime_error(util::Formatter() << "Identity Constraint " << constr->getName() << " ptr " << constr << " is out of band : start_idx + nrows > data_.lb_.size()");
            }
        }
        else
        {
            if(constr->getControlVariable() == ControlVariable::ExternalWrench)
            {
                start_idx += iAwrench * 6;
                iAwrench++;
            }

            if(start_idx + nrows <= data_.lb_.size() )
            {
                if(constr->isActivated())
                {
                    data_.A_.block(row_idx,start_idx,nrows,ncols) = constr->getConstraintMatrix();
                    data_.lbA_.segment(row_idx,nrows) = data_.lbA_.segment(row_idx,nrows).cwiseMax(constr->getLowerBound());
                    data_.ubA_.segment(row_idx,nrows) = data_.ubA_.segment(row_idx,nrows).cwiseMin(constr->getUpperBound());
                }
                else
                {
                    data_.A_.block(row_idx,start_idx,nrows,ncols).setZero();
                }
                // Increment rows in A by the constraint number of rows
                row_idx += nrows;
            }
            else
            {
                // Error
                throw std::runtime_error(util::Formatter() << "Constraint " << constr->getName() << " ptr " << constr << " is out of band : start_idx + nrows > data_.lb_.size()");
            }
        }
    }
}
