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

#include <orca/optim/ControlVariable.h>

#include <orca/task/GenericTask.h>
#include <orca/constraint/GenericConstraint.h>

#include <orca/common/TaskCommon.h>
#include <orca/common/Mutex.h>
#include <orca/common/Wrench.h>

#include <orca/optim/QPSolver.h>
#include <map>
#include <list>
#include <algorithm>

namespace orca
{
namespace optim
{

    class OptimVector
    {
    public:
        void addInRegister(std::shared_ptr<QPSolver> qp);
        void addInRegister(std::shared_ptr<common::TaskCommon> t);

        void removeFromRegister(std::shared_ptr<QPSolver> qp);
        void removeFromRegister(std::shared_ptr<common::TaskCommon> t) ;

        void buildControlVariablesMapping(int ndof);

        void print() const;

        const std::list< std::shared_ptr<common::Wrench> > getWrenches() const;
        const std::list< std::shared_ptr<task::GenericTask> > getTasks() const;
        const std::list< std::shared_ptr<constraint::GenericConstraint> > getConstraints() const;

        int getNrOfWrenches() const;

        int getIndex(ControlVariable var) const;

        int getSize(ControlVariable var) const;

        int getTotalSize() const;

        bool hasFloatingBaseVars() const;

        int getNrOfDegreesOfFreedom() const;

        int ConfigurationSpaceDimension() const;

    private:
        mutable MutexRecursive mutex;

        void resizeTasks();
        void resizeConstraints();

        int ndof_ = 0;
        int nwrenches_ = 0;
        bool is_floating_base_ = true;

        std::map<ControlVariable, unsigned int > idx_map_;
        std::map<ControlVariable, unsigned int > size_map_;

        std::list< std::shared_ptr<common::Wrench> > wrenches_;
        std::list< std::shared_ptr<common::TaskCommon> > commons_;

        std::list< std::shared_ptr<task::GenericTask> > tasks_;
        std::list< std::shared_ptr<constraint::GenericConstraint> > constraints_;

        std::shared_ptr<QPSolver> qp_;
    };

    OptimVector& OptimisationVector();

}
}
