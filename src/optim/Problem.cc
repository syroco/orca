#include "orca/optim/Problem.h"
#include "orca/common/Wrench.h"
#include "orca/utils/Utils.h"
#include "orca/common/TaskBase.h"
#include "orca/task/GenericTask.h"
#include "orca/task/RegularisationTask.h"
#include "orca/task/WrenchTask.h"
#include "orca/constraint/GenericConstraint.h"
#include "orca/constraint/DynamicsEquationConstraint.h"
#include "orca/robot/RobotModel.h"

using namespace orca::common;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::math;
using namespace orca::utils;
using namespace orca::robot;

Problem::Problem(std::shared_ptr<RobotModel> robot, QPSolver::SolverType solver_type)
: qpsolver_(std::make_shared<QPSolver>(solver_type))
, robot_(robot)
{
    this->ndof_ = robot->getNrOfDegreesOfFreedom();
    // initialise the problem as we have  the ndof
    resize();
}

Problem::~Problem()
{

}

void Problem::print() const
{
    std::cout << "Problem data : " << std::endl;
    data_.print();
    std::cout << std::endl;
    std::cout << "Optimization Vector : " << '\n';
    const auto all_variables =
    {
          ControlVariable::X
        , ControlVariable::GeneralisedAcceleration
        , ControlVariable::FloatingBaseAcceleration
        , ControlVariable::JointAcceleration
        , ControlVariable::GeneralisedTorque
        , ControlVariable::FloatingBaseWrench
        , ControlVariable::JointTorque
        , ControlVariable::ExternalWrench
        , ControlVariable::ExternalWrenches
        , ControlVariable::Composite
        , ControlVariable::None
    };
    for(auto v : all_variables)
    {
        std::cout << "  - " << v << " index " << getIndex(v) << " size " << getSize(v) << '\n';
    }
    std::cout << "Problem objects : " << std::endl;
    std::cout << "      Tasks" << std::endl;
    for(auto t : this->tasks_)
    {
        std::cout << "          [" << t->getState() << "] " << t->getName() << std::endl;
    }
    std::cout << "      Constraints" << std::endl;
    for(auto c : this->constraints_)
    {
        std::cout << "          [" << c->getState() << "] " << c->getName() << std::endl;
    }
}

const std::list< std::shared_ptr< const Wrench > >& Problem::getWrenches() const
{
    return wrenches_;
}

const std::list< std::shared_ptr< GenericTask > >& Problem::getTasks() const
{
    return tasks_;
}

const std::list< std::shared_ptr< GenericConstraint > >& Problem::getConstraints() const
{
    return constraints_;
}

void Problem::resizeSolver(int nvar,int nconstr)
{
    LOG_INFO << "Resizing QPSolver to nvars " << nvar
            << " nconstr " << nconstr;
    qpsolver_->resize(nvar,nconstr);
}

std::shared_ptr<QPSolver> Problem::qpSolver()
{
    return qpsolver_;
}

unsigned int Problem::computeNumberOfConstraintRows() const
{
    int number_of_constraints_rows = 0;

    for(auto constraint : getConstraints())
    {
        if(constraint->getConstraintMatrix().isIdentity())
        {
            LOG_DEBUG << "For constraint " << constraint->getName() << " : detecting lb < x < ub constraint, not adding rows";
            // Detecting lb < x < ub constraint
        }
        else
        {
            LOG_DEBUG << "For constraint " << constraint->getName() << " : adding constraint " << constraint->rows() << " rows";
            number_of_constraints_rows += constraint->rows();
            LOG_DEBUG << "For constraint " << constraint->getName() << " : number of constraint rows is now  " << number_of_constraints_rows ;
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

bool Problem::addTask(std::shared_ptr<GenericTask> task)
{
    // TODO: Add more checks for more return values
    if(!exists(task,tasks_))
    {
        LOG_INFO << "Adding task " << task->getName();
        tasks_.push_back(task);
        if(task->hasWrench())
        {
            if(addWrench(task->getWrench()))
            {
                resize();
                return true;
            }
            else
            {
                return false;
            }
        }
        resize();
        return true;
    }
    else
    {
        LOG_WARNING << "Task " << task->getName() << " is already present in the problem";
        return false;
    }
}

bool Problem::constraintExists(std::shared_ptr< constraint::GenericConstraint> cstr)
{
    return exists(cstr,constraints_);
}

bool Problem::taskExists(std::shared_ptr< task::GenericTask> task)
{
    return exists(task,tasks_);
}

bool Problem::addWrench(std::shared_ptr< const Wrench > wrench)
{
    if(!exists(wrench,wrenches_))
    {
        LOG_INFO << "Adding Wrench " << wrench->getName();
        wrenches_.push_back(wrench);
        return true;
    }
    else
    {
        LOG_WARNING << "Wrench " << wrench->getName() << " is already present in the problem";
        return false;
    }
    return false;
}

bool Problem::addConstraint(std::shared_ptr<GenericConstraint> constraint)
{
    if(!exists(constraint,constraints_))
    {
        LOG_INFO << "Adding constraint " << constraint->getName();
        constraints_.push_back(constraint);
        if(constraint->hasWrench())
        {
            if(addWrench(constraint->getWrench()))
            {
                resize();
                return true;
            }
            else
            {
                return false;
            }

        }
        resize();
        return true;
    }
    else
    {
        LOG_WARNING << "Constraint " << constraint->getName() << " is already present in the problem";
        return false;
    }
}

void Problem::resize()
{
    if(ndof_ == 0)
        orca_throw("Cannot resize if ndof is 0");

    int nvars = this->mapping_.generate(ndof_,wrenches_.size());
    int nconstr = computeNumberOfConstraintRows();

    if(nvars != this->number_of_variables_ || nconstr != this->number_of_constraints_rows_)
    {
        LOG_INFO << "Resizing problem to (" << Size(nvars,nconstr) << ") , previously was (" << Size(nvars,nconstr) << ")";
        this->number_of_variables_ = nvars;
        this->number_of_constraints_rows_ = nconstr;
        resizeProblemData(this->number_of_variables_,this->number_of_constraints_rows_);
        resizeSolver(this->number_of_variables_,this->number_of_constraints_rows_);

        // Only resize tasks that depends on the problem changing size
        // TODO : account for joint changes
        for(auto t : tasks_)
            if(t->dependsOnProblem())
                t->resize();
        for(auto c : constraints_)
            if(c->dependsOnProblem())
                c->resize();
    }
}

bool Problem::solve()
{
    return qpsolver_->solve(data_);
}

ReturnCode Problem::getReturnCode() const
{
    return qpsolver_->getReturnCode();
}

Eigen::VectorXd Problem::getSolution(ControlVariable var) const
{
    return data_.primal_solution_.segment(getIndex(var),getSize(var));
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
    // Build Objective
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
            if(task->isComputing())
            {
                data_.H_.block(start_idx, start_idx, nrows, ncols).noalias()  += task->getWeight() * task->getQuadraticCost().getHessian();
                data_.g_.segment(start_idx ,ncols).noalias()                  += task->getWeight() * task->getQuadraticCost().getGradient();
            }
        }
        else
        {
            // Error
            task->print();
            orca_throw(Formatter() << "Task " << task->getName() << " ptr " << task << " << block of size (" << Size(nrows,ncols) << ")"
                      << "\nCould not fit at index [" << start_idx << ":" << start_idx << "]"
                      << " because H size is (" << Size(data_.H_) << ")");
        }
    }
    // build constraints as
    // lbA < Ax < ubA
    // l < x < u <---- identity constraints
    int iAwrench = 0;
    iwrench = 0;
    int row_idx = 0;
    for(auto constraint : constraints_)
    {
        int start_idx = getIndex(constraint->getControlVariable());

        int nrows = constraint->rows();
        int ncols = constraint->cols();

        if(constraint->getConstraintMatrix().isIdentity())
        {
            if(constraint->getControlVariable() == ControlVariable::ExternalWrench)
            {
                start_idx += iwrench * 6;
                iwrench++;
            }

            if(start_idx + nrows <= data_.lb_.size() )
            {
                if(constraint->isComputing())
                {
                    data_.lb_.segment(start_idx ,nrows) = data_.lb_.segment(start_idx ,nrows).cwiseMax(constraint->getLowerBound());
                    data_.ub_.segment(start_idx ,nrows) = data_.ub_.segment(start_idx ,nrows).cwiseMin(constraint->getUpperBound());
                }
            }
            else
            {
                // Error
                constraint->print();
                orca_throw(Formatter() << "Identity Constraint " << constraint->getName() << " ptr " << constraint << " is out of band : start_idx + nrows > data_.lb_.size()");
            }
        }
        else
        {
            if(constraint->getControlVariable() == ControlVariable::ExternalWrench)
            {
                start_idx += iAwrench * 6;
                iAwrench++;
            }

            if(start_idx + nrows <= data_.lb_.size()
                && row_idx + nrows <= data_.A_.rows()
                && start_idx + ncols <= data_.A_.cols() )
            {
                if(constraint->isComputing())
                {
                    data_.A_.block(row_idx,start_idx,nrows,ncols) = constraint->getConstraintMatrix();
                    data_.lbA_.segment(row_idx,nrows) = data_.lbA_.segment(row_idx,nrows).cwiseMax(constraint->getLowerBound());
                    data_.ubA_.segment(row_idx,nrows) = data_.ubA_.segment(row_idx,nrows).cwiseMin(constraint->getUpperBound());
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
                constraint->print();
                if(start_idx + nrows <= data_.lb_.size())
                    orca_throw(Formatter() << "Constraint " << constraint->getName() << " is out of band : start_idx + nrows > data_.lb_.size()");
                if(row_idx + nrows <= data_.A_.rows())
                    orca_throw(Formatter() << "Constraint " << constraint->getName() << " is out of band : row_idx + nrows <= data_.A_.rows()");
                if(start_idx + ncols <= data_.A_.cols())
                    orca_throw(Formatter() << "Constraint " << constraint->getName() << " is out of band : start_idx + ncols <= data_.A_.cols()");
            }
        }
    }
}
