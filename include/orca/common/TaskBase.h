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

#include "orca/robot/RobotDynTree.h"
#include "orca/optim/ControlVariable.h"
#include "orca/common/Mutex.h"
#include "orca/optim/ControlProblem.h"

namespace orca
{
    
namespace common
{
    class TaskBase
    {
    public:
        TaskBase(optim::ControlVariable control_var);
        virtual ~TaskBase();
        
        void setRobotModel(std::shared_ptr<robot::RobotDynTree> robot);

        bool loadRobotModel(const std::string& file_url);

        optim::ControlVariable getControlVariable() const;

        void setName(const std::string& name);

        const std::string& getName() const;

        virtual void update() = 0;
        virtual void resize() = 0;
        virtual void print() const = 0;

        /**
         * @brief Activates the constraint in the solver. Otherwise its -inf < 0.x < inf
         *
         */
        virtual bool activate();

        /**
         * @brief Check if the constraint is active in the solver
         *
         * @return bool
         */
        virtual bool isActivated() const;
        /**
         * @brief Desactivates the constraint : in the solver it is seen as -inf < 0.x < inf
         *
         */
        virtual bool desactivate();
        /**
        * @brief Check if the constraint is inserted in the problem
        *
        * @return bool
        */
        virtual bool setProblem(std::shared_ptr< orca::optim::Problem > problem, bool insert = true);
        virtual bool insertInProblem();
        virtual bool removeFromProblem();
        
        /**
        * @brief Insert the constraint in the QP problem
        *
        */
        bool isInitialized() const;
        
        /**
        * @brief The recursive mutex to protect public fucntions
        *
        */
        mutable common::MutexRecursive mutex;
        
        std::shared_ptr<robot::RobotDynTree> robot();
        std::shared_ptr<optim::Problem> problem();
        
        bool hasProblem() const;
        bool hasRobot() const;
        void printStateIfErrors() const;
        
    private:
        bool is_activated_ = true;
        std::string name_;
        std::shared_ptr<optim::Problem> problem_;
        std::shared_ptr<robot::RobotDynTree> robot_;
        optim::ControlVariable control_var_;
    };
} // namespace common
} // namespace orca
