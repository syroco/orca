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
#include "orca/utils/Utils.h"
#include "orca/utils/Logger.h"
#include "orca/task/GenericTask.h"
#include "orca/optim/ControlVariable.h"
#include "orca/optim/Problem.h"
#include "orca/optim/ResolutionStrategy.h"
#include "orca/optim/QPSolver.h"
#include "orca/robot/RobotDynTree.h"
#include "orca/constraint/GenericConstraint.h"

namespace orca
{
namespace optim
{
    class Controller
    {
    public:
        Controller(const std::string& name
            , std::shared_ptr<robot::RobotDynTree> robot
            ,ResolutionStrategy resolution_strategy
            ,QPSolver::SolverType solver_type);

        void print() const;

        void setPrintLevel(int level);

        const std::string& getName();

        std::shared_ptr<robot::RobotDynTree> robot();

        void setRobotModel(std::shared_ptr<robot::RobotDynTree> robot);

        bool update(double current_time, double dt);

        common::ReturnCode getReturnCode() const;

        bool addTask(std::shared_ptr<task::GenericTask> task);

        bool addConstraint(std::shared_ptr<constraint::GenericConstraint> cstr);

        const Eigen::VectorXd& getSolution();

        Eigen::VectorXd getJointTorqueCommand();

        Eigen::VectorXd getJointAccelerationCommand();

        void activateAll(double current_time);

        void deactivateAll(double current_time);

        bool allDeactivated();

    private:
        void insertNewProblem();

        void updateTasks(double current_time, double dt);

        void updateConstraints(double current_time, double dt);

        std::list< std::shared_ptr<Problem> > problems_;

        std::shared_ptr<robot::RobotDynTree> robot_;

        Eigen::VectorXd joint_torque_command_;
        Eigen::VectorXd joint_acceleration_command_;

        ResolutionStrategy resolution_strategy_;
        QPSolver::SolverType solver_type_;
        const std::string name_;
    };
} // namespace optim
} //namespace orca
