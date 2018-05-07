#include "orca/optim/Problem.h"
#include "orca/common/Wrench.h"
#include "orca/utils/Utils.h"
#include "orca/common/TaskBase.h"
#include "orca/task/GenericTask.h"
#include "orca/constraint/GenericConstraint.h"

using namespace orca::common;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::math;
using namespace orca::utils;

Problem::Problem(QPSolver::SolverType solver_type)
: qpsolver_(std::make_shared<QPSolver>(solver_type))
{
}

Problem::~Problem()
{

}

void Problem::print() const
{
    std::cout << "Problem data : " << std::endl;
    data_.print();
    std::cout << std::endl;
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

const std::list< std::shared_ptr< const Wrench> >& Problem::getWrenches() const
{
    return wrenches_;
}

const std::list< std::shared_ptr<GenericTask> >& Problem::getTasks() const
{
    return tasks_;
}

const std::list< std::shared_ptr<GenericConstraint> >& Problem::getConstraints() const
{
    return constraints_;
}

void Problem::resizeSolver(int nvar,int nconstr)
{
    LOG_INFO << "Resizing QPSolver to nvars " << nvar
            << " nconstr " << nconstr;
    qpsolver_->resize(nvar,nconstr);
}


unsigned int Problem::computeNumberOfConstraintRows( std::list<std::shared_ptr<GenericConstraint> > constraints) const
{
    int number_of_constraints_rows = 0;

    for(auto constr : constraints)
    {
        if(constr->getConstraintMatrix().isIdentity())
        {
            LOG_DEBUG << "For constraint " << constr->getName() << " : detecting lb < x < ub constraint, not adding rows";
            // Detecting lb < x < ub constraint
        }
        else
        {
            LOG_DEBUG << "For constraint " << constr->getName() << " : adding constraint " << constr->rows() << " rows";
            number_of_constraints_rows += constr->rows();
            LOG_DEBUG << "For constraint " << constr->getName() << " : number of constraint rows is now  " << number_of_constraints_rows ;
        }
    }
    //LOG_DEBUG << "We are now at  " << data_.H_.rows() << "x" << data_.A_.rows();
    LOG_DEBUG << "Total number of constraint rows : " << number_of_constraints_rows;
    return number_of_constraints_rows;
}

void Problem::resizeProblemData(int nvar,int nconstr)
{
    data_.resize(nvar,nconstr);
}

void Problem::setProblemObjects(
    int ndof
    , std::list< std::shared_ptr< GenericTask > > tasks
    , std::list< std::shared_ptr< GenericConstraint > > constraints
    , std::list< std::shared_ptr< const Wrench > > wrenches
)
{
    this->tasks_ = tasks;
    this->constraints_ = constraints;
    this->wrenches_ = wrenches;

    this->number_of_variables_ = this->mapping_.generate(ndof,wrenches_.size());
    this->number_of_constraints_rows_ = computeNumberOfConstraintRows(constraints_);

    resizeProblemData(this->number_of_variables_,this->number_of_constraints_rows_);
    resizeSolver(this->number_of_variables_,this->number_of_constraints_rows_);
}

bool Problem::solve()
{
    return qpsolver_->solve(data_) == 0;
}

const Eigen::VectorXd& Problem::getSolution() const
{
    return data_.primal_solution_;
}

unsigned int Problem::getIndex(ControlVariable var) const
{
    return mapping_.getIndex(var);
}
unsigned int Problem::getSize(ControlVariable var) const
{
    return mapping_.getSize(var);
}
unsigned int Problem::getTotalSize() const
{
    return mapping_.getTotalSize();
}
const std::map<ControlVariable, unsigned int >& Problem::getIndexMap() const
{
    return mapping_.getIndexMap();
}
const std::map<ControlVariable, unsigned int >& Problem::getSizeMap() const
{
    return mapping_.getSizeMap();
}
void Problem::build()
{
    // Reset H and g
    data_.reset();

    int iwrench = 0;
    for(auto task : tasks_)
    {
        int start_idx = getIndex(task->getControlVariable());

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
            throw std::runtime_error(Formatter() << "Task " << task->getName() << " ptr " << task << " << block of size (" << Size(nrows,ncols) << ")"
                      << "\nCould not fit at index [" << start_idx << ":" << start_idx << "]"
                      << " because H size is (" << Size(data_.H_) << ")");
        }
    }

    int iAwrench = 0;
    iwrench = 0;
    int row_idx = 0;
    for(auto constr : constraints_)
    {
        int start_idx = getIndex(constr->getControlVariable());

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
                throw std::runtime_error(Formatter() << "Identity Constraint " << constr->getName() << " ptr " << constr << " is out of band : start_idx + nrows > data_.lb_.size()");
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
                throw std::runtime_error(Formatter() << "Constraint " << constr->getName() << " ptr " << constr << " is out of band : start_idx + nrows > data_.lb_.size()");
            }
        }
    }
}
