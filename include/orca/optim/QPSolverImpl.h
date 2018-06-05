#pragma once
#include "orca/common/ReturnCode.h"
#include "orca/optim/ProblemData.h"
#include "orca/utils/Utils.h"

namespace orca
{
namespace optim
{

class QPSolverImpl
{
public:
    virtual void resize(int nvar,int nconstr) = 0;
    virtual void setPrintLevel(int level) = 0;
    virtual void setDefaultOptions() = 0;
    virtual common::ReturnCode solve(ProblemData& data ) = 0;
    virtual void getPrimalSolution(Eigen::VectorXd& solution) = 0;
};

} // namespace optim
} // namespace orca
