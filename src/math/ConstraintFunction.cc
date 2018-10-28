#include "orca/math/ConstraintFunction.h"
#include "orca/utils/Utils.h"

namespace orca
{
namespace math
{

using namespace orca::utils;

void ConstraintFunction::print() const
{
    std::cout << "Constraint Matrix C size " << getSize() << std::endl;
    std::cout << getConstraintMatrix() << std::endl;
    std::cout << "  Lower Bound lb" << std::endl;
    std::cout << getLowerBound() << std::endl;
    std::cout << "  Upper Bound ub" << std::endl;
    std::cout << getUpperBound() << std::endl;
}

const Eigen::VectorXd& ConstraintFunction::getLowerBound() const
{
    return lower_bound_;
}

const Eigen::VectorXd& ConstraintFunction::getUpperBound() const
{
    return upper_bound_;
}

void ConstraintFunction::setLowerBound(const Eigen::VectorXd& newlb)
{
    if(newlb.size() == lower_bound_.size())
    {
        lower_bound_ = newlb;
    }
    else
    {
        LOG_ERROR << "Size of lower bound do not match, provided "
            << newlb.size() << ", but expected " << lower_bound_.size();
    }
}

void ConstraintFunction::setUpperBound(const Eigen::VectorXd& newub)
{
    if(newub.size() == upper_bound_.size())
    {
        upper_bound_ = newub;
    }
    else
    {
        LOG_ERROR << "Size of upper bound do not match, provided "
            << newub.size() << ", but expected " << upper_bound_.size();
    }
}

void ConstraintFunction::setConstraintMatrix(const Eigen::MatrixXd& newC)
{
    if(Size(newC) == getSize())
    {
        C_ = newC;
    }
    else
    {
        LOG_ERROR << "Size of constraint matrix do not match, provided "
            << Size(newC) << ", but expected " << getSize();
    }
}

Eigen::VectorXd& ConstraintFunction::lowerBound()
{
    return lower_bound_;
}

Eigen::VectorXd& ConstraintFunction::upperBound()
{
    return upper_bound_;
}

const Eigen::MatrixXd& ConstraintFunction::getConstraintMatrix() const
{
    return C_;
}

Eigen::MatrixXd& ConstraintFunction::constraintMatrix()
{
    return C_;
}

void ConstraintFunction::reset()
{
    C_.setZero();
    math::setToHighest(upper_bound_);
    math::setToLowest(lower_bound_);
}

void ConstraintFunction::resize(int new_rows,int new_cols)
{
    const int old_rows = C_.rows();
    const int old_cols = C_.cols();

    bool rows_changed = ( old_rows != new_rows );
    bool cols_changed = ( old_cols != new_cols );

    if(! rows_changed && ! cols_changed)
    {
        // No Need to resize anything
        return;
    }

    // Add new rows only
    if( rows_changed && ! cols_changed )
    {
        // Keep old data, and add zeros if matrix is growing
        C_.conservativeResize(new_rows,Eigen::NoChange);

        // If matrix if growing
        if(new_rows > old_rows)
        {
            // Initialize the new rows at zero
            C_.block(old_rows,0,new_rows - old_rows,old_cols).setZero();
            lower_bound_.conservativeResizeLike( Eigen::VectorXd::Constant(new_rows,   - math::Infinity) );
            upper_bound_.conservativeResizeLike( Eigen::VectorXd::Constant(new_rows,     math::Infinity) );
        }
        else
        {
            // If we remove rows
            lower_bound_.conservativeResize(new_rows);
            lower_bound_.conservativeResize(new_rows);
        }
    }
    // Add new cols only
    // Size of lower/upper band does not change
    if( ! rows_changed && cols_changed )
    {
        C_.conservativeResize(Eigen::NoChange,new_cols);
        if(new_cols > old_cols)
        {
            // Initialize the new cols at zero
            C_.block(0,old_cols,old_rows,new_cols-old_cols).setZero();
        }
    }

    // Add both
    if( rows_changed && cols_changed )
    {
        C_.conservativeResizeLike( Eigen::MatrixXd::Zero(new_rows,new_cols) );
        lower_bound_.conservativeResizeLike( Eigen::VectorXd::Constant(new_rows,   - math::Infinity) );
        upper_bound_.conservativeResizeLike( Eigen::VectorXd::Constant(new_rows,     math::Infinity) );
        // add rows and remove cols
        // if(new_rows > old_rows && new_cols < old_cols)
        // {
        //
        // }
        // // remove rows and remove cols
        // if(new_rows < old_rows && new_cols < old_cols)
        // {
        //
        // }
        // // remove rows and add cols
        // if(new_rows < old_rows && new_cols > old_cols)
        // {
        //
        // }
        // // add rows and add cols
        // if(new_rows > old_rows && new_cols > old_cols)
        // {
        //
        // }
    }
}

Size ConstraintFunction::getSize() const
{
    return Size(C_.rows(),C_.cols());
}

int ConstraintFunction::rows() const
{
    return C_.rows();
}

int ConstraintFunction::cols() const
{
    return C_.cols();
}

} // namespace math
} // namespace orca
