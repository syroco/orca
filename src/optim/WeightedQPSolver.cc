#include <orca/optim/WeightedQPSolver.h>
#include <orca/optim/OptimisationVector.h>

using namespace orca::optim;
using namespace orca::task;
using namespace orca::constraint;
WeightedQPSolver::WeightedQPSolver()
{
    OptimisationVector().addInRegister(this);
}

WeightedQPSolver::~WeightedQPSolver()
{
    OptimisationVector().removeFromRegister(this);
}

void WeightedQPSolver::resize()
{
    MutexLock lock(mutex);
    
    LOG_DEBUG << "WeightedQPSolver::resize()";
    
    constraints_ =  OptimisationVector().getConstraints();
    tasks_ = OptimisationVector().getTasks();
    idx_map_ = OptimisationVector().getIndexMap();
    size_map_ = OptimisationVector().getSizeMap();
    
    //MutexLock lock(mutex);
    //LOG_DEBUG << "WeightedQPSolver::resize() Mutex lock, resizing";
    const int nvars = size_map_[ControlVariable::X];
    LOG_DEBUG << "WeightedQPSolver::resize() nvars " << nvars;
    int number_of_constraints_rows = 0;
    
    LOG_DEBUG << "WeightedQPSolver::resize() getConstraints " << nvars;
    LOG_DEBUG << "WeightedQPSolver::resize() got constraints " << nvars;
    
    for(auto constr : constraints_)
    {
        LOG_DEBUG << "WeightedQPSolver::resize() for constr " << constr ;
        if(constr->getConstraintMatrix().isIdentity())
        {
            LOG_DEBUG << "WeightedQPSolver::resize() Detecting lb < x < ub constraint " << constr;
            // Detecting lb < x < ub constraint
            //std::cout << "Detecting lb < x < ub constraint" << std::endl;
        }
        else
        {
            LOG_DEBUG << "WeightedQPSolver::resize() qdding constrqint rows ";
            number_of_constraints_rows += constr->rows();
            LOG_DEBUG << "WeightedQPSolver::resize()number of rows is now  " << number_of_constraints_rows ;
        }
    }
    LOG_DEBUG << "WeightedQPSolver::resize() resizeinternal " << nvars << "x" << number_of_constraints_rows;
    
    QPSolver::resizeInternal(nvars,number_of_constraints_rows);
}

void WeightedQPSolver::buildOptimisationProblem()
{
    MutexTryLock lock(mutex);
    
    if(!lock.isSuccessful())
    {
        LOG_DEBUG << "Mutex locked, not updating";
        return;
    }
    // Reset H and g
    data_.reset();

    int iwrench = 0;
    for(auto task : tasks_)
    {
        MutexLock lock(task->mutex);
        
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
            // TODO: move isActivated in the task update
            if(task->isActivated())
            {
                data_.H_.block(start_idx, start_idx, nrows, ncols).noalias()  += task->getWeight() * task->getQuadraticCost().getHessian();
                data_.g_.segment(start_idx ,ncols).noalias()                  += task->getWeight() * task->getQuadraticCost().getGradient();
            }
        }
        else
        {
            // Error
            throw std::runtime_error(util::Formatter() << "Task " << task->getName() << " ptr " << task << " is out of band : start_idx + nrows <= data_.H_.rows() && start_idx + ncols <= data_.H_.cols()");
        }
    }

    int iAwrench = 0;
    iwrench = 0;
    int row_idx = 0;
    for(auto constr : constraints_)
    {
        MutexLock lock(constr->mutex);
        
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
                // TODO: move isActivated in the constraints
                if(constr->isActivated())
                {
                    data_.lb_.segment(start_idx ,nrows) = data_.lb_.segment(start_idx ,nrows).cwiseMax(constr->getLowerBound());
                    data_.ub_.segment(start_idx ,nrows) = data_.ub_.segment(start_idx ,nrows).cwiseMin(constr->getUpperBound());
                }
                else
                {
                    // No nothing
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
