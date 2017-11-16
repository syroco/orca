#define EIGEN_NO_MALLOC

#include <cstdlib>
#include <Eigen/Core>

#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree
{
    
    Eigen::Vector3d diff(const iDynTree::Rotation& R_a_b1, const iDynTree::Rotation& R_a_b2)
    {
        typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;
        
        iDynTree::Rotation R_b1_b2 = R_a_b1.inverse() * R_a_b2;
        Eigen::AngleAxisd aa(Eigen::Map<const Matrix3dRowMajor>(R_b1_b2.data()));
        return toEigen(R_a_b1) * aa.angle() * aa.axis();
    }
    
    Eigen::Matrix<double,6,1> diff(const iDynTree::Transform& t_a_b1, const iDynTree::Transform& t_a_b2)
    {
        
        typedef Eigen::Matrix<double,6,1> Vector6d;
        Vector6d dX;
        
        dX.head<3>() = iDynTree::toEigen(t_a_b2.getPosition() - t_a_b1.getPosition());
        dX.tail<3>() = iDynTree::diff(t_a_b1.getRotation(),t_a_b2.getRotation());
        
        return dX;
    }
    
}

Eigen::Vector3d diffRot(const Eigen::Matrix3d& R_a_b1, const Eigen::Matrix3d& R_a_b2)
{
    Eigen::Matrix3d R_b1_b2 = R_a_b1.transpose() * R_a_b2;
    Eigen::AngleAxisd aa(Eigen::Map<const Eigen::Matrix3d>(R_b1_b2.data()));
    return R_a_b1 * aa.angle() * aa.axis();
}

Eigen::Matrix<double,6,1> diffTransform(const Eigen::Matrix4d& t_a_b1, const Eigen::Matrix4d& t_a_b2)
{
    Eigen::Matrix<double,6,1> d_t1_t2;
    
    d_t1_t2.head<3>() = t_a_b2.block<3,1>(0,3) - t_a_b1.block<3,1>(0,3);
    d_t1_t2.tail<3>() = diffRot(t_a_b1.topLeftCorner<3,3>(),t_a_b2.topLeftCorner<3,3>());
    return d_t1_t2;
}


int main(int argc, char** argv)
{
    Eigen::MatrixXd z;
    z.resize(10,0);
    iDynTree::Transform a(iDynTree::RotationRaw::RPY(0,0,0), iDynTree::Position(1,1,1)),b(iDynTree::RotationRaw::RPY(0,1.57,0), iDynTree::Position(-1,-1,-1));
    auto eig_a = iDynTree::toEigen(a.asHomogeneousTransform());
    
    std::cout << "Eigen a " << std::endl << eig_a << std::endl;
    
    std::cout << "a " << std::endl << a.toString() << std::endl;
    std::cout << "b " << std::endl << b.toString() << std::endl;
    
    auto c = iDynTree::diff(a.getRotation(),b.getRotation());
    std::cout << "c1 " << c << std::endl;
    std::cout << "c2 " << iDynTree::diff(a,b) << std::endl;
    
    std::cout << "c3 " << diffTransform(iDynTree::toEigen(a.asHomogeneousTransform()),iDynTree::toEigen(b.asHomogeneousTransform())) 
    << std::endl;

    return 0;
}
