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

#include "orca/utils/Utils.h"
#include "orca/math/AffineFunction.h"

namespace orca
{
namespace math
{

class WeightedEuclidianNormFunction : public AffineFunction
{
public:
    WeightedEuclidianNormFunction();

    class QuadraticCost
    {
    public:
        void resize(int A_or_b_rows);

        Size getSize() const;

        int rows() const;

        int cols() const;

        void computeQuadraticCost(const Eigen::VectorXd& SelectionVector
                                , const Eigen::MatrixXd& Weight
                                , const Eigen::MatrixXd& A
                                , const Eigen::VectorXd& b);

        void computeHessian(const Eigen::VectorXd& SelectionVector
                        , const Eigen::MatrixXd& Weight
                        , const Eigen::MatrixXd& A);

        void computeGradient(const Eigen::VectorXd& SelectionVector
                        , const Eigen::MatrixXd& Weight
                        , const Eigen::MatrixXd& A
                        , const Eigen::VectorXd& b);

        const Eigen::MatrixXd& getHessian() const;
        const Eigen::VectorXd& getGradient() const;

    private:
        Eigen::MatrixXd Hessian_;
        Eigen::VectorXd Gradient_;
    };

    void print() const;

    void setWeight(const Eigen::MatrixXd& weight);

    void setWeight(double weight);

    void computeQuadraticCost();

    const Eigen::VectorXd& getSelectionVector() const;

    void setSelectionVector(const Eigen::VectorXd& s);

    const Eigen::MatrixXd& getWeight() const;

    const QuadraticCost& getQuadraticCost() const;

    void resize(int rows,int cols);

private:
    Eigen::MatrixXd Weight_;
    Eigen::VectorXd SelectionVector_;
    QuadraticCost quadCost_;
};

}
}
