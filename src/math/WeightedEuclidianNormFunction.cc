#include <orca/math/WeightedEuclidianNormFunction.h>
using namespace orca::math;

WeightedEuclidianNormFunction::WeightedEuclidianNormFunction()
{
    
}

void WeightedEuclidianNormFunction::QuadraticCost::resize(int A_or_b_rows)
{
    Hessian_.setZero(A_or_b_rows,A_or_b_rows);
    Gradient_.setZero(A_or_b_rows);
}

Size WeightedEuclidianNormFunction::QuadraticCost::getSize() const
{
    return Size(Hessian_.rows(),Hessian_.cols());
}

int WeightedEuclidianNormFunction::QuadraticCost::rows() const
{
    return Hessian_.rows();
}

int WeightedEuclidianNormFunction::QuadraticCost::cols() const
{
    return Hessian_.cols();
}

void WeightedEuclidianNormFunction::QuadraticCost::computeQuadraticCost(const Eigen::VectorXd& SelectionVector
                                                , const Eigen::MatrixXd& Weight
                                                , const Eigen::MatrixXd& A
                                                , const Eigen::VectorXd& b)
{
    computeHessian(SelectionVector,Weight,A);
    computeGradient(SelectionVector,Weight,A,b);
}

void WeightedEuclidianNormFunction::QuadraticCost::computeHessian(const Eigen::VectorXd& SelectionVector
                                                , const Eigen::MatrixXd& Weight
                                                , const Eigen::MatrixXd& A)
{
    Hessian_.noalias() = SelectionVector.asDiagonal() * Weight * A.transpose() * A ;
}

void WeightedEuclidianNormFunction::QuadraticCost::computeGradient(const Eigen::VectorXd& SelectionVector
                                                , const Eigen::MatrixXd& Weight
                                                , const Eigen::MatrixXd& A
                                                , const Eigen::VectorXd& b)
{
    Gradient_.noalias() =  2.0 * SelectionVector.asDiagonal() * Weight * A.transpose() * b ;
}

const Eigen::MatrixXd& WeightedEuclidianNormFunction::QuadraticCost::getHessian() const
{
    return Hessian_;
}

const Eigen::VectorXd& WeightedEuclidianNormFunction::QuadraticCost::getGradient() const 
{
    return Gradient_;
}

void WeightedEuclidianNormFunction::print() const
{
    std::cout << "WeightedEuclidianNormFunction SelectionVector || Ax + b ||weight" << std::endl;
    AffineFunction::print();
    std::cout << "Weight" << std::endl;
    std::cout << getWeight() << std::endl;
    std::cout << "SelectionVector" << std::endl;
    std::cout << getSelectionVector() << std::endl;
    std::cout << "Hessian" << std::endl;
    std::cout << getQuadraticCost().getHessian() << std::endl;
    std::cout << "Gradient" << std::endl;
    std::cout << getQuadraticCost().getGradient() << std::endl;
}

void WeightedEuclidianNormFunction::setWeight(const Eigen::MatrixXd& weight)
{
    if(Size(weight) == Size(Weight_))
    {
        Weight_ = weight;
    }
    else
    {
        throw std::runtime_error("Size of weight matix do not match");
    }
}

void WeightedEuclidianNormFunction::setWeight(double weight)
{
    Weight_.setIdentity();
    Weight_.diagonal() *= weight;
}

void WeightedEuclidianNormFunction::computeQuadraticCost()
{
    quadCost_.computeQuadraticCost(SelectionVector_,Weight_,A_,b_);
}

const Eigen::VectorXd& WeightedEuclidianNormFunction::getSelectionVector() const 
{
    return SelectionVector_;
}

const Eigen::MatrixXd& WeightedEuclidianNormFunction::getWeight() const
{
    return Weight_;
}

const WeightedEuclidianNormFunction::QuadraticCost& WeightedEuclidianNormFunction::getQuadraticCost() const
{
    return quadCost_;
}

void WeightedEuclidianNormFunction::resize(int rows,int cols)
{
    AffineFunction::resize(rows,cols);
    quadCost_.resize(cols);

    SelectionVector_.conservativeResizeLike(Eigen::VectorXd::Ones(cols));

    Weight_.conservativeResizeLike(Eigen::MatrixXd::Identity(cols,cols));
    //Weight_.conservativeResize(cols,cols);
    //Weight_.setIdentity();

    this->computeQuadraticCost();
}

