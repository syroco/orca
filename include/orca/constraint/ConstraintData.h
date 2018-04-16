#pragma once

#include <Eigen/Core>
#include "orca/optim/ControlVariable.h"

namespace orca
{
namespace constraint
{
    
    struct ConstraintData
    {
        std::string name;
        optim::ControlVariable control_var;
        int rows;
        int cols;
        Eigen::MatrixXd constraint_matrix;
        Eigen::VectorXd lower_bound;
        Eigen::VectorXd upper_bound;
    };
    
    inline void resize(ConstraintData& cdata, int rows, int cols)
    {
        cdata.rows = rows;
        cdata.cols = cols;
        cdata.constraint_matrix.resize(rows,cols);
        cdata.lower_bound.resize(rows);
        cdata.upper_bound.resize(rows);
    }
}
}

