// This file is a part of the ORCA framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// ORCA is a whole-body reactive controller framework for robotics.
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/


#pragma once
#include <Eigen/Dense>
#include <limits>

namespace orca
{
namespace math
{
    static const double Infinity = std::numeric_limits<double>::infinity();

    inline void setToHighest(Eigen::VectorXd& v)
    {
        v.setConstant(   math::Infinity );
    }

    inline void setToLowest(Eigen::VectorXd& v)
    {
        v.setConstant( - math::Infinity );
    }

    typedef Eigen::Matrix<double,6,1> Vector6d;

    inline Eigen::Vector3d diffRot(const Eigen::Matrix3d& R_a_b1, const Eigen::Matrix3d& R_a_b2)
    {
        Eigen::Matrix3d R_b1_b2 = R_a_b1.transpose() * R_a_b2;
        Eigen::AngleAxisd aa(Eigen::Map<const Eigen::Matrix3d>(R_b1_b2.data()));
        return R_a_b1 * aa.angle() * aa.axis();
    }

    inline Eigen::Matrix<double,6,1> diffTransform(const Eigen::Matrix4d& t_a_b1, const Eigen::Matrix4d& t_a_b2)
    {
        Eigen::Matrix<double,6,1> d_t1_t2;

        d_t1_t2.head<3>() = t_a_b2.block<3,1>(0,3) - t_a_b1.block<3,1>(0,3);
        d_t1_t2.tail<3>() = diffRot(t_a_b1.topLeftCorner<3,3>(),t_a_b2.topLeftCorner<3,3>());
        return d_t1_t2;
    }

    inline Eigen::Quaterniond quatFromRPY(double roll,double pitch,double yaw )
    {
        return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    }

    inline Eigen::Quaterniond quatFromKukaConvention(double A,double B,double C )
    {
        return Eigen::AngleAxisd(A, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(B, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(C, Eigen::Vector3d::UnitX());
    }

    struct Size
    {
        template <typename Derived> Size(const Eigen::MatrixBase<Derived>& mat)
        : Size(mat.rows(),mat.cols())
        {}

        Size(int rows = 0, int cols = 0)
        : rows_(rows)
        , cols_(cols)
        {}
        Size(const Size& s)
        : Size(s.rows(),s.cols())
        {}
        bool operator==(const Size& s) const
        {
            return (s.rows() == rows_) && (s.cols() == cols_);
        }
        bool operator!=(const Size& s) const
        {
            return !(*this == s);
        }
        int rows() const { return rows_;}
        int cols() const { return cols_;}
    private:
        int rows_ = 0;
        int cols_ = 0;
    };
} // namespace math
} // namespace orca
