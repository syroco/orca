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
#include "orca/math/Utils.h"
#include "orca/util/Utils.h"
#include "orca/util/Logger.h"
#include "orca/optim/ControlVariable.h"
#include <map>
#include <list>

namespace orca
{
    namespace common
    {
        class Wrench;
    }
    namespace task
    {
        class GenericTask;
    }
    namespace constraint
    {
        class GenericConstraint;
    }
    namespace robot
    {
        class RobotDynTree;
    }
}



namespace orca
{
namespace optim
{

struct ProblemData
{
    void resize(int nvar, int nconstr)
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

class MultiObjectiveOptimisationProblem
{
public:
    virtual bool solve() = 0;
    bool addTask(std::shared_ptr<orca::task::GenericTask> task);

    bool taskExists(std::shared_ptr<orca::task::GenericTask> task);

    bool constraintExists(std::shared_ptr<orca::constraint::GenericConstraint> cstr);

    void setRobotModel(std::shared_ptr<orca::robot::RobotDynTree> robot);

    void print();

    const std::list< std::shared_ptr<orca::common::Wrench> >& getWrenches() const;

    const std::list< std::shared_ptr<orca::task::GenericTask> >& getTasks() const;

    const std::list< std::shared_ptr<orca::constraint::GenericConstraint> >& getConstraints() const;

    int getConfigurationSpaceDimension() const;
    
    const std::map<ControlVariable, unsigned int >& getIndexMap() const;
    
    const std::map<ControlVariable, unsigned int >& getSizeMap() const;
    
    int getIndex(ControlVariable var) const;

    int getSize(ControlVariable var) const;

    int getTotalSize() const;
protected:
    std::list< std::shared_ptr<orca::common::Wrench> > wrenches_;
    std::list< std::shared_ptr<orca::task::GenericTask> > tasks_;
    std::list< std::shared_ptr<orca::constraint::GenericConstraint> > constraints_;
    int ndof_ = 0;
    int configuration_space_dim_ = 0;
    bool is_floating_base_ = true;
    int nwrenches_ = 0;

    std::map<ControlVariable, unsigned int > index_map_;
    std::map<ControlVariable, unsigned int > size_map_;
};

class WeightedProblem : public MultiObjectiveOptimisationProblem
{
public:
    bool solve()
    {

        return true;
    }
};


} // namespace optim
} // namespace orca
