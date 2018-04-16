#include "orca/optim/QPSolver.h"
#include "orca/util/Logger.h"
#include "QPSolver_qpOASES.impl"
#include <iostream>
using namespace orca::optim;

QPSolver::QPSolver(QPSolver::SolverType type)
{
    switch(type)
    {
        case SolverType::qpOASES:
            pimpl = std::unique_ptr<SolverImpl<qpOASES> >(new SolverImpl<qpOASES>);
            break;
        default:
            throw "Solver not implemented";
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

int QPSolver::solve(ProblemData& data)
{   
    int ret = pimpl->solve(data);
    pimpl->getPrimalSolution(data.primal_solution_);
    return ret;
}
