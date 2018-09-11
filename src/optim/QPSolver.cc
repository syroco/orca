#include "orca/optim/QPSolver.h"
#include "orca/utils/Utils.h"
#include "QPSolverImpl_qpOASES.impl"
// #include "QPSolverImpl_osqp.impl"
#include "QPSolverImpl_eigQuadProg.impl"
#include <iostream>
using namespace orca::optim;
using namespace orca::utils;
using namespace orca::common;

QPSolver::QPSolver(QPSolver::SolverType solver_type)
{
    switch(solver_type)
    {
        case SolverType::qpOASES:
            pimpl = make_unique<QPSolverImpl_qpOASES>();
            break;
        // case SolverType::osqp:
        //     pimpl = make_unique<QPSolverImpl_osqp>();
        //     break;
        case SolverType::eigQuadProg:
            pimpl = make_unique<QPSolverImpl_eigQuadProg>();
            break;
        default:
            orca_throw(Formatter() << "QPSolver " << solver_type << " not implemented");
    }
}

QPSolver::~QPSolver()
{}

void QPSolver::setPrintLevel(int level)
{
    // PL_DEBUG_ITER = -2, PL_TABULAR, PL_NONE, PL_LOW, PL_MEDIUM, PL_HIGH
    pimpl->setPrintLevel(level);
}

void QPSolver::resize(int nvar,int nconstr)
{
    pimpl->resize(nvar,nconstr);
}

ReturnCode QPSolver::getReturnCode() const
{
    return ret_;
}

bool QPSolver::solve(ProblemData& data)
{
    ret_ = pimpl->solve(data);
    pimpl->getPrimalSolution(data.primal_solution_);
    return ret_ == ReturnCode::SUCCESSFUL_RETURN;
}
