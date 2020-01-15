/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_EIGEN_HELPERS_H
#define IDYNTREE_EIGEN_HELPERS_H

#include <Eigen/Dense>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Utils.h>

#if __cplusplus > 199711L
#include <iDynTree/Core/SparseMatrix.h>
#endif


namespace iDynTree
{
    //Useful typedefs
    //TODO: change methods below to use these typedefs
    typedef Eigen::Map<Eigen::VectorXd> iDynTreeEigenVector;
    typedef Eigen::Map<const Eigen::VectorXd> iDynTreeEigenConstVector;
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> iDynTreeEigenMatrix;
    typedef const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> iDynTreeEigenConstMatrix;
    typedef Eigen::Map<iDynTreeEigenMatrix> iDynTreeEigenMatrixMap;
    typedef Eigen::Map<iDynTreeEigenConstMatrix> iDynTreeEigenConstMatrixMap;

#if __cplusplus > 199711L
    template<typename>
    struct is_sparsematrix : std::false_type {};

    template<iDynTree::MatrixStorageOrdering ordering>
    struct is_sparsematrix<iDynTree::SparseMatrix<ordering>> : std::true_type {};
#endif



// Dynamics size toEigen methods
inline Eigen::Map<Eigen::VectorXd> toEigen(VectorDynSize & vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(),vec.size());
}

inline Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(MatrixDynSize & mat)
{
    return Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(mat.data(),mat.rows(),mat.cols());
}

inline Eigen::Map<const Eigen::VectorXd> toEigen(const VectorDynSize & vec)
{
    return Eigen::Map<const Eigen::VectorXd>(vec.data(),vec.size());
}

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
inline Eigen::Map<const Eigen::VectorXd> toEigen(Span<const double> vec)
{
    return Eigen::Map<const Eigen::VectorXd>(vec.data(),vec.size());
}

inline Eigen::Map<Eigen::VectorXd> toEigen(iDynTree::Span<double> vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(),vec.size());
}
#endif

inline Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > toEigen(const MatrixDynSize & mat)
{
    return Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(mat.data(),mat.rows(),mat.cols());
}

// Fixed size toEigen methods
template<unsigned int VecSize>
inline Eigen::Map<Eigen::Matrix<double,VecSize,1> > toEigen(VectorFixSize<VecSize> & vec)
{
    return Eigen::Map<Eigen::Matrix<double,VecSize,1> >(vec.data(),vec.size());
}

template<unsigned int VecSize>
inline Eigen::Map<const Eigen::Matrix<double,VecSize,1> > toEigen(const VectorFixSize<VecSize> & vec)
{
    return Eigen::Map<const Eigen::Matrix<double,VecSize,1> >(vec.data());
}

template<unsigned int nRows, unsigned int nCols>
inline Eigen::Map< Eigen::Matrix<double,nRows,nCols,Eigen::RowMajor> > toEigen(MatrixFixSize<nRows,nCols> & mat)
{
    return Eigen::Map< Eigen::Matrix<double,nRows,nCols,Eigen::RowMajor> >(mat.data());
}

template<unsigned int nRows>
inline Eigen::Map< Eigen::Matrix<double,nRows,1> > toEigen(MatrixFixSize<nRows,1> & mat)
{
    return Eigen::Map< Eigen::Matrix<double,nRows,1> >(mat.data());
}

template<unsigned int nCols>
inline Eigen::Map< Eigen::Matrix<double,1,nCols> > toEigen(MatrixFixSize<1, nCols> & mat)
{
    return Eigen::Map< Eigen::Matrix<double,1, nCols> >(mat.data());
}

template<unsigned int nRows, unsigned int nCols>
inline Eigen::Map< const Eigen::Matrix<double,nRows,nCols,Eigen::RowMajor> > toEigen(const MatrixFixSize<nRows,nCols> & mat)
{
    return Eigen::Map< const Eigen::Matrix<double,nRows,nCols,Eigen::RowMajor> >(mat.data());
}

// Spatial vectors
inline const Eigen::Matrix<double,6,1> toEigen(const SpatialMotionVector & vec)
{
    Eigen::Matrix<double,6,1> ret;

    ret.segment<3>(0) = toEigen(vec.getLinearVec3());
    ret.segment<3>(3) = toEigen(vec.getAngularVec3());

    return ret;
}

inline const Eigen::Matrix<double,6,1> toEigen(const SpatialForceVector & vec)
{
    Eigen::Matrix<double,6,1> ret;

    ret.segment<3>(0) = toEigen(vec.getLinearVec3());
    ret.segment<3>(3) = toEigen(vec.getAngularVec3());

    return ret;
}

// Spatial vectors
inline void fromEigen(SpatialMotionVector & vec, const Eigen::Matrix<double,6,1> & eigVec)
{
    toEigen(vec.getLinearVec3()) = eigVec.segment<3>(0);
    toEigen(vec.getAngularVec3()) = eigVec.segment<3>(3);
}

inline void fromEigen(SpatialForceVector & vec, const Eigen::Matrix<double,6,1> & eigVec)
{
    toEigen(vec.getLinearVec3()) = eigVec.segment<3>(0);
    toEigen(vec.getAngularVec3()) = eigVec.segment<3>(3);
}

inline void fromEigen(Transform & trans, const Eigen::Matrix4d & eigMat)
{
    Rotation rot;
    Position pos;

    toEigen(rot) = eigMat.block<3,3>(0,0);
    toEigen(pos) = eigMat.block<3,1>(0,3);

    trans.setRotation(rot);
    trans.setPosition(pos);
}

template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor> skew(const Eigen::MatrixBase<Derived> & vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor>() << 0.0, -vec[2], vec[1],
                                                                              vec[2], 0.0, -vec[0],
                                                                             -vec[1], vec[0], 0.0).finished();
}

template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> unskew(const Eigen::MatrixBase<Derived> & mat)
{
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 1>() << mat(2,1), mat(0,2), mat(1,0) ).finished();
}


/**
 * Submatrix helpers
 */
template<unsigned int nRows, unsigned int nCols, typename iDynTreeMatrixType>
inline void setSubMatrix(iDynTreeMatrixType& mat,
                         const IndexRange rowRange,
                         const IndexRange colRange,
                         const MatrixFixSize<nRows,nCols>& subMat)
{
#if __cplusplus > 199711L
    static_assert(!iDynTree::is_sparsematrix<iDynTreeMatrixType>::value, "You cannot set a subMatrix on a SparseMatrix.");
#endif

    toEigen(mat).block(rowRange.offset,colRange.offset,rowRange.size,colRange.size) = toEigen(subMat);
    return;
}

template<typename iDynTreeMatrixType, typename EigMatType>
inline void setSubMatrix(iDynTreeMatrixType& mat,
                         const IndexRange rowRange,
                         const IndexRange colRange,
                         const EigMatType& subMat)
{
#if __cplusplus > 199711L
    static_assert(!iDynTree::is_sparsematrix<iDynTreeMatrixType>::value, "You cannot set a subMatrix on a SparseMatrix.");
#endif
    toEigen(mat).block(rowRange.offset,colRange.offset,rowRange.size,colRange.size) = subMat;
    return;
}

template<typename iDynTreeMatrixType>
inline void setSubMatrix(iDynTreeMatrixType& mat,
                         const IndexRange rowRange,
                         const IndexRange colRange,
                         const double subMat)
{
#if __cplusplus > 199711L
    static_assert(!iDynTree::is_sparsematrix<iDynTreeMatrixType>::value, "You cannot set a subMatrix on a SparseMatrix.");
#endif
    assert(rowRange.size == 1);
    assert(colRange.size == 1);
    mat(rowRange.offset,colRange.offset) = subMat;
    return;
}

template<typename iDynTreeMatrixType>
inline void setSubMatrixToIdentity(iDynTreeMatrixType& mat,
                                   const IndexRange rowRange,
                                   const IndexRange colRange)
{
#if __cplusplus > 199711L
    static_assert(!iDynTree::is_sparsematrix<iDynTreeMatrixType>::value, "You cannot set a setSubMatrixToIdentity on a SparseMatrix.");
#endif
    assert(rowRange.size == colRange.size);
    for(int i=0; i < rowRange.size; i++)
    {
        mat(rowRange.offset+i,colRange.offset+i) = 1.0;
    }
    return;
}

template<typename iDynTreeMatrixType>
inline void setSubMatrixToMinusIdentity(iDynTreeMatrixType& mat,
                                        const IndexRange rowRange,
                                        const IndexRange colRange)
{
#if __cplusplus > 199711L
    static_assert(!iDynTree::is_sparsematrix<iDynTreeMatrixType>::value, "You cannot set a setSubMatrixToMinusIdentity on a SparseMatrix.");
#endif
    assert(rowRange.size == colRange.size);
    for(int i=0; i < rowRange.size; i++)
    {
        mat(rowRange.offset+i,colRange.offset+i) = -1.0;
    }
    return;
}


template<unsigned int size>
inline void setSubVector(VectorDynSize& vec,
                         const IndexRange range,
                         const VectorFixSize<size>& subVec)
{
    toEigen(vec).segment(range.offset,range.size) = toEigen(subVec);
    return;
}

inline void setSubVector(VectorDynSize& vec,
                         const IndexRange range,
                         double subVec)
{
    assert(range.size==1);
    vec(range.offset) = subVec;
    return;
}

inline void setSubVector(VectorDynSize& vec,
                         const IndexRange range,
                         const SpatialMotionVector& twist)
{
    assert(range.size==6);
    toEigen(vec).segment(range.offset,range.size) = toEigen(twist);
    return;
}

inline void setSubVector(VectorDynSize& vec,
                         const IndexRange range,
                         const SpatialForceVector& wrench)
{
    assert(range.size==6);
    toEigen(vec).segment(range.offset,range.size) = toEigen(wrench);
    return;
}

template<typename T>
inline void setSubVector(VectorDynSize& vec,
                         const IndexRange range,
                         const T& subVec)
{
    toEigen(vec).segment(range.offset,range.size) = subVec;
    return;
}

}

#endif /* IDYNTREE_EIGEN_HELPERS_H */
