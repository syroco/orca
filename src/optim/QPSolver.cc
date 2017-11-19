#include <orca/optim/QPSolver.h>
#include <orca/optim/OptimisationVector.h>
#include <qpOASES.hpp>
#include <iostream>
using namespace orca::optim;

struct QPSolver::SolverImpl
{
    std::unique_ptr<qpOASES::SQProblem> qpoases_;
    qpOASES::Options options_;
    bool qpoases_initialized_ = false;

    void resize(int nvar,int nconstr)
    {
        if(!qpoases_)
        {
            LOG_INFO << "[QPSolver] New qpOASES::SQProblem (nvar=" << nvar << ",nconstr=" <<nconstr<< ")" ;
            qpoases_.reset(new qpOASES::SQProblem(nvar,nconstr,qpOASES::HST_POSDEF));
            setDefaultOptions();

            qpoases_initialized_ = false;
        }
        else if(nvar != qpoases_->getNV() || nconstr != qpoases_->getNC() )
        {
            LOG_INFO << "[QPSolver] Resize qpOASES::SQProblem (nvar=" << nvar << ",nconstr=" <<nconstr<< ")" ;
            options_ = qpoases_->getOptions();
            qpoases_.reset(new qpOASES::SQProblem(nvar,nconstr,qpOASES::HST_POSDEF));
            qpoases_->setOptions(options_);

            qpoases_initialized_ = false;
        }
    }

    void setPrintLevel(int level)
    {
        if(!qpoases_) return;

        switch (level)
        {
            case -2:    qpoases_->setPrintLevel(qpOASES::PL_DEBUG_ITER);  break;
            case -1:    qpoases_->setPrintLevel(qpOASES::PL_TABULAR);     break;
            case  0:    qpoases_->setPrintLevel(qpOASES::PL_NONE);        break;
            case  1:    qpoases_->setPrintLevel(qpOASES::PL_LOW);         break;
            case  2:    qpoases_->setPrintLevel(qpOASES::PL_MEDIUM);      break;
            case  3:    qpoases_->setPrintLevel(qpOASES::PL_HIGH);        break;
            default: break;
        }
    }

    void setDefaultOptions()
    {
        if(!qpoases_) return;

        options_.setToMPC(); // setToReliable() // setToDefault()
        options_.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
        options_.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
        qpoases_->setOptions( options_ );
    }

    int solve(QPSolverData& data )
    {
        if(!qpoases_) return qpOASES::RET_NOTHING_TO_DO;

        int nWSR = 1000000;
        qpOASES::returnValue ret;

        if(!qpoases_initialized_)
        {
            // Initialise the problem, once it has found a solution, we can hotstart
            ret = qpoases_->init(   data.H_.data()
                                        , data.g_.data()
                                        , data.A_.data()
                                        , data.lb_.data()
                                        , data.ub_.data()
                                        , data.lbA_.data()
                                        , data.ubA_.data()
                                        , nWSR
//                                         , 0 // cputime
//                                         , 0 // xOpt
//                                         , 0 // yOpt
//                                         , &guessedBounds
                                        );

            // Keep init if it didn't work
            if(ret == qpOASES::SUCCESSFUL_RETURN)
                qpoases_initialized_ = true;
        }
        else
        {
            // Otherwise let's reuse the previous solution to find a solution faster
            ret = qpoases_->hotstart( data.H_.data()
                                        , data.g_.data()
                                        , data.A_.data()
                                        , data.lb_.data()
                                        , data.ub_.data()
                                        , data.lbA_.data()
                                        , data.ubA_.data()
                                        , nWSR);
            if(ret != qpOASES::SUCCESSFUL_RETURN)
                qpoases_initialized_ = false;

        }
        return ret;
    }
    void getPrimalSolution(Eigen::VectorXd& solution)
    {
        if(!qpoases_)
        {
            throw std::runtime_error("qpsolver pointer is null");
        }
        int nv = qpoases_->getNV();
        if(solution.size() != nv)
        {
            std::stringstream err;
            err << "Provided vector size is " << solution.size() << ",but the number of variable is " << nv << std::endl;
            throw std::runtime_error(err.str());
        }
        qpoases_->getPrimalSolution(solution.data());
    }
};

QPSolver::QPSolver()
: pimpl(new SolverImpl())
{

}

QPSolver::~QPSolver()
{

}

void QPSolver::print()
{
    std::cout << "=========================================================================================================================================" << std::endl;
    std::cout << "H" << std::endl;
    std::cout << data_.H_ << std::endl;
    std::cout << "g" << std::endl;
    std::cout << data_.g_ << std::endl;
    std::cout << "A" << std::endl;
    std::cout << data_.A_ << std::endl;
    std::cout << "lbA" << std::endl;
    std::cout << data_.lbA_ << std::endl;
    std::cout << "ubA" << std::endl;
    std::cout << data_.ubA_ << std::endl;
    std::cout << "lb" << std::endl;
    std::cout << data_.lb_ << std::endl;
    std::cout << "ub" << std::endl;
    std::cout << data_.ub_ << std::endl;
    std::cout << "=========================================================================================================================================" << std::endl;
}

void QPSolver::setPrintLevel(int level)
{
    // PL_DEBUG_ITER = -2, PL_TABULAR, PL_NONE, PL_LOW, PL_MEDIUM, PL_HIGH
    pimpl->setPrintLevel(level);
}

void QPSolver::resizeInternal(int nvar,int nconstr)
{
    data_.resize(nvar,nconstr);
    pimpl->resize(nvar,nconstr);
}

int QPSolver::solve()
{
    return pimpl->solve(data_);
}

const Eigen::VectorXd& QPSolver::getPrimalSolution()
{
    pimpl->getPrimalSolution(data_.primal_solution_);
    return data_.primal_solution_;
}
