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
#include "orca/robot/RobotDynTree.h"
#include "orca/math/ConstraintFunction.h"
#include "orca/common/TaskBase.h"

namespace orca
{
namespace constraint
{

/**
 * @brief Builds a double bounded function : l < C.x < u
 * With x the control variable, l = lowerBound,
 * u = upperBound, and C the constraint matrix.
 *
 */
class GenericConstraint : public common::TaskBase
{
public:
    /**
     * @brief Construct the double bounded function, and set its default state : activated.
     * The lowerBound is set to -inf and the upper bound to +inf
     * C is set to Zero by default.
     *
     * @param control_var The control variable it depends on
     */
    GenericConstraint(const std::string& name,optim::ControlVariable control_var);

    /**
     * @brief Destructor. Also removes from problem if still active/inserted
     *
     */
    virtual ~GenericConstraint();

    virtual void print() const;

    /**
     * @brief Get the size of the constraint matrix (rows,cols)
     *
     * @return orca::math::Size
     */
    math::Size getSize() const;

    /**
     * @brief Get the number of rows of the constraint matrix
     *
     * @return int
     */
    int rows() const;

    /**
     * @brief Get the number of column (it should be the size of the control variable chosen) of the constraint matrix
     *
     * @return int
     */
    int cols() const;

    /**
     * @brief Returns the lower bound from the constraint function (alias)
     *
     * @return const Eigen::VectorXd&
     */
    virtual const Eigen::VectorXd& getLowerBound() const;

    /**
     * @brief Returns the upperbound from the constraint function (alias)
     *
     * @return const Eigen::VectorXd&
     */
    virtual const Eigen::VectorXd& getUpperBound() const;

    /**
     * @brief Returns the constraint matrix from the constraint function (alias)
     *
     * @return const Eigen::MatrixXd&
     */
    virtual const Eigen::MatrixXd& getConstraintMatrix() const;

    /**
     * @brief Get the underlying constraint function containing the 2 bound vectors and the constraint matrix
     *
     * @return const orca::math::ConstraintFunction&
     */
    const math::ConstraintFunction& getConstraintFunction() const;


protected:
    virtual void onResize() = 0;
    virtual void onCompute(double current_time, double dt);
    virtual void onUpdateConstraintFunction(double current_time, double dt) = 0;
    /**
     * @brief Replace the constraint matrix with a new one
     *
     * @param newC The new constraint matrix
     */
    void setConstraintMatrix(const Eigen::MatrixXd& newC);

    /**
     * @brief Replace the lower bound with a new one
     *
     * @param low The new lowerbound
     */
    void setLowerBound(const Eigen::VectorXd& low);

    /**
     * @brief Replace the upper bound with a new one
     *
     * @param up The new upperbound
     */
    void setUpperBound(const Eigen::VectorXd& up);

    /**
     * @brief Returns a reference to the constraint matrix from the constraint function (alias)
     *
     * @return Eigen::MatrixXd&
     */
    Eigen::MatrixXd& constraintMatrix();
    Eigen::VectorXd& upperBound();
    Eigen::VectorXd& lowerBound();

    /**
     * @brief Get a reference the underlying constraint function containing the 2 bound vectors and the constraint matrix
     *
     * @return orca::math::ConstraintFunction&
     */
    math::ConstraintFunction& constraintFunction();

private:
    math::ConstraintFunction constraint_function_;
};

}
}
