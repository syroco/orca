// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
//
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
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

#pragma once

#include <memory>
#include <orca/math/Utils.h>
#include <orca/common/Mutex.h>

namespace orca
{
namespace optim
{
struct QPSolverData
{
    void resize(int nvar, int nconstr)
    {
        if(nvar != H_.cols() || nconstr != A_.rows() )
        {
            H_.conservativeResizeLike(Eigen::MatrixXd::Zero(nvar,nvar));
            g_.conservativeResizeLike(Eigen::VectorXd::Zero(nvar));

            A_.conservativeResizeLike(Eigen::MatrixXd::Zero(nconstr,nvar));

            lbA_.conservativeResizeLike( Eigen::VectorXd::Constant(nconstr, - math::Infinity) );
            lb_.conservativeResizeLike(  Eigen::VectorXd::Constant(nvar,    - math::Infinity) );

            ubA_.conservativeResizeLike( Eigen::VectorXd::Constant(nconstr,  math::Infinity) );
            ub_.conservativeResizeLike(  Eigen::VectorXd::Constant(nvar,     math::Infinity) );

            primal_solution_.conservativeResizeLike(Eigen::VectorXd::Zero(nvar));
        }
    }

    void reset()
    {
        H_.setZero();
        g_.setZero();
        lb_.setConstant( - math::Infinity );
        ub_.setConstant(   math::Infinity );
        A_.setZero();
        lbA_.setConstant( - math::Infinity );
        ubA_.setConstant(   math::Infinity );
        primal_solution_.setZero();
    }

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
    Eigen::VectorXd g_;
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    Eigen::VectorXd lbA_;
    Eigen::VectorXd ubA_;
    Eigen::VectorXd primal_solution_;
};


class QPSolver
{
public:
    QPSolver();
    virtual ~QPSolver();
    void setPrintLevel(int level);
    const Eigen::VectorXd& getPrimalSolution();
    virtual void buildOptimisationProblem() = 0;
    virtual void resize() = 0;
    void print() const;
    int solve();
protected:
    QPSolverData data_;
    void resizeInternal(int nvar, int nconstr);
    mutable MutexRecursive mutex;
private:
    struct SolverImpl; std::shared_ptr<SolverImpl> pimpl;
};

}
}
