#include "orca/optim/QPSolver.h"
#include "orca/utils/Utils.h"
#include "QPSolverImpl_qpOASES.impl"
// #include "QPSolverImpl_osqp.impl"
//#include "QPSolverImpl_eigQuadProg.impl"
#include <iostream>
using namespace orca::optim;
using namespace orca::utils;
using namespace orca::common;

QPSolver::QPSolver()
{
    // Create a default solver
    setImplementationType(solver_type_);
}

bool QPSolver::setImplementationType(QPSolverImplType solver_type)
{
    if(solver_type_ == solver_type)
        return true;

    // We need resize if we previously had a pimpl
    // And the size makes sens
    bool need_resize = (bool(pimpl_) && nvar_ > 0 && nconstr_ > 0 );
    
    switch(solver_type)
    {
        case QPSolverImplType::qpOASES:
            pimpl_ = make_unique<QPSolverImpl_qpOASES>();
            break;
        // case SolverType::osqp:
        //     pimpl_ = make_unique<QPSolverImpl_osqp>();
        //     break;
//         case QPSolverImplType::quadprog:
//             pimpl_ = make_unique<QPSolverImpl_eigQuadProg>();
//             break;
        default:
            orca_throw(Formatter() << "QPSolver '" << QPSolverImplTypetoString(solver_type) << "' not implemented");
    }
    
    solver_type_ = solver_type;
    
    if(need_resize)
        resize(nvar_,nconstr_);
    
    return bool(pimpl_);
}

QPSolver::~QPSolver()
{}

void QPSolver::setPrintLevel(int level)
{
    // PL_DEBUG_ITER = -2, PL_TABULAR, PL_NONE, PL_LOW, PL_MEDIUM, PL_HIGH
    pimpl_->setPrintLevel(level);
}

void QPSolver::resize(unsigned int nvar,unsigned int nconstr)
{
    nvar_ = nvar;
    nconstr_ = nconstr;
    if(nvar > 0 && nconstr > 0)
        pimpl_->resize(nvar,nconstr);
}

ReturnCode QPSolver::getReturnCode() const
{
    return ret_;
}

bool QPSolver::solve(ProblemData& data)
{
    ret_ = pimpl_->solve(data);
    pimpl_->getPrimalSolution(data.primal_solution_);
    return ret_ == ReturnCode::SUCCESSFUL_RETURN;
}
