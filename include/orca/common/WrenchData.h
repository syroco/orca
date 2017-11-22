#pragma once

#include <Eigen/Core>
#include <orca/optim/ControlVariable.h>

namespace orca
{
namespace common
{
    
    struct WrenchData
    {
        std::string name;
        int rows;
        int cols;
        bool is_initialized;
        bool is_activated;
        Eigen::MatrixXd jacobian_;
        Eigen::MatrixXd jacobian_transpose_;
    };
    
    inline void resize(WrenchData& cdata, int rows, int cols)
    {
        cdata.rows = rows;
        cdata.cols = cols;
        cdata.jacobian_.resize(rows,cols);
        cdata.jacobian_transpose_.resize(rows,cols);
    }
}
}
