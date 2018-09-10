#include "orca/math/AffineFunction.h"

using namespace orca::math;

void AffineFunction::print() const
{
    std::cout << "Ax+b" << std::endl;
    std::cout << "A size "<< getSize()<< std::endl;
    std::cout << getA() << std::endl;
    std::cout << "b^t size " << getb().size() << std::endl;
    std::cout << getb().transpose() << std::endl;
}

void AffineFunction::resize(int rows,int cols)
{
    A_.conservativeResizeLike(Eigen::MatrixXd::Zero(rows,cols));
    b_.conservativeResizeLike(Eigen::VectorXd::Zero(rows));
}

void AffineFunction::setA(const Eigen::MatrixXd& newA)
{
    if(Size(newA) != Size(A_))
    {
        std::cerr << "Size mismatched, expected matrix of size "
            << Size(A_) << " but you provided a matrix of size "
            << Size(newA) <<'\n';
    }
    A_ = newA;
}

void AffineFunction::setb(const Eigen::VectorXd& newb)
{
    if(Size(newb) != Size(b_))
    {
        std::cerr << "Size mismatched, expected vector of size "
            << b_.size() << " but you provided a vector of size "
            << newb.size() <<'\n';
    }
    b_ = newb;
}

const Eigen::MatrixXd& AffineFunction::getA() const
{
    return A_;
}

const Eigen::VectorXd& AffineFunction::getb() const
{
    return b_;
}

Eigen::MatrixXd& AffineFunction::A()
{
    return A_;
}

Eigen::VectorXd& AffineFunction::b()
{
    return b_;
}

Size AffineFunction::getSize() const
{
    return Size(A_.rows(),A_.cols());
}

int AffineFunction::cols() const
{
    return A_.cols();
}

int AffineFunction::rows() const
{
    return A_.rows();
}
