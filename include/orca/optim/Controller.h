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
#include "orca/optim/ControlVariable.h"
#include "orca/optim/Problem.h"
#include "orca/optim/ResolutionStrategy.h"
#include "orca/optim/QPSolver.h"
#include "orca/robot/RobotDynTree.h"
#include "orca/common/Wrench.h"
#include "orca/common/TaskBase.h"
#include "orca/task/GenericTask.h"
#include "orca/task/RegularisationTask.h"
#include "orca/task/WrenchTask.h"
#include "orca/constraint/GenericConstraint.h"
#include "orca/constraint/Contact.h"
#include <map>
#include <list>

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
            ,QPSolver::SolverType solver_type)
        : name_(name)
        , robot_(robot)
        , resolution_strategy_(resolution_strategy)
        , solver_type_(solver_type)
        {
            if(resolution_strategy != ResolutionStrategy::OneLevelWeighted)
            {
                throw std::runtime_error(utils::Formatter() << "Only ResolutionStrategy::OneLevelWeighted is supported for now");
            }
            insertNewProblem();
        }
        void setPrintLevel(int level)
        {
            for(auto problem : problems_)
            {
                problem->qpSolver()->setPrintLevel(level);
            }
        }

        const std::string& getName()
        {
            return name_;
        }

        std::shared_ptr<robot::RobotDynTree> robot()
        {
            if(!robot_)
                throw std::runtime_error(utils::Formatter() << "Robot is not set");
            return robot_;
        }

        void setRobotModel(std::shared_ptr<robot::RobotDynTree> robot)
        {
            robot_ = robot;
        }

        void update(double current_time, double dt)
        {
            switch (resolution_strategy_)
            {
                case ResolutionStrategy::OneLevelWeighted:
                    updateTasks(current_time,dt);
                    updateConstraints(current_time,dt);
                    problems_.front()->build();
                    problems_.front()->solve();
                break;
                default:
                    throw std::runtime_error(utils::Formatter() << "unsupported resolution strategy");
            }
        }

        bool addTask(std::shared_ptr<task::GenericTask> task)
        {
            if(resolution_strategy_ == ResolutionStrategy::OneLevelWeighted)
            {
                task->setRobotModel(robot_);
                task->setProblem(problems_.front());
                return problems_.front()->addTask(task) != Problem::Error;
            }
            return false;
        }

        bool addConstraint(std::shared_ptr<constraint::GenericConstraint> cstr)
        {
            if(resolution_strategy_ == ResolutionStrategy::OneLevelWeighted)
            {
                cstr->setRobotModel(robot_);
                cstr->setProblem(problems_.front());
                return problems_.front()->addConstraint(cstr) != Problem::Error;
            }
            return false;
        }

        Eigen::VectorXd getFullSolution()
        {
            switch (resolution_strategy_)
            {
                case ResolutionStrategy::OneLevelWeighted:
                    return problems_.front()->getSolution(ControlVariable::X);
                default:
                    throw std::runtime_error(utils::Formatter() << "Unsupported resolution strategy");
            }
        }

        Eigen::VectorXd getJointTorqueCommand()
        {
            switch (resolution_strategy_)
            {
                case ResolutionStrategy::OneLevelWeighted:
                    return problems_.front()->getSolution(ControlVariable::JointSpaceTorque);
                default:
                    throw std::runtime_error(utils::Formatter() << "Unsupported resolution strategy");
            }
        }

        Eigen::VectorXd getJointAccelerationCommand()
        {
            switch (resolution_strategy_)
            {
                case ResolutionStrategy::OneLevelWeighted:
                    return problems_.front()->getSolution(ControlVariable::JointSpaceAcceleration);
                default:
                    throw std::runtime_error(utils::Formatter() << "Unsupported resolution strategy");
            }
        }

        void activateAll(double current_time)
        {
            for(auto problem : problems_)
            {
                for(auto t : problem->getTasks())
                {
                    t->activate();
                }
                for(auto c : problem->getConstraints())
                {
                    c->activate();
                }
            }
        }

        void deactivateAll(double current_time)
        {
            for(auto problem : problems_)
            {
                for(auto t : problem->getTasks())
                {
                    t->deactivate();
                }
                for(auto c : problem->getConstraints())
                {
                    c->deactivate();
                }
            }
        }

        bool allDeactivated()
        {
            for(auto problem : problems_)
            {
                for(auto t : problem->getTasks())
                {
                    if(t->getState() != common::TaskBase::Deactivated)
                        return false;
                }
                for(auto c : problem->getConstraints())
                {
                    if(c->getState() != common::TaskBase::Deactivated)
                        return false;
                }
            }
            return true;
        }

    protected:
        void insertNewProblem()
        {
            LOG_INFO << "Creating new problem at level " << problems_.size();
            auto problem = std::make_shared<Problem>(robot_,solver_type_);
            problem->qpSolver()->setPrintLevel(0);

            auto dynamics_equation = std::make_shared<constraint::DynamicsEquationConstraint>("DynamicsEquation");
            auto global_regularisation = std::make_shared<task::RegularisationTask<ControlVariable::X> >("GlobalRegularisation");

            dynamics_equation->setRobotModel(robot_);
            dynamics_equation->setProblem(problem);

            global_regularisation->setRobotModel(robot_);
            global_regularisation->setProblem(problem);

            global_regularisation->euclidianNorm().setWeight(1E-5);

            problem->addConstraint(dynamics_equation);
            problem->addTask(global_regularisation);

            problems_.push_back(problem);
        }
        
        void updateTasks(double current_time, double dt)
        {
            for(auto problem : problems_)
            {
                for(auto t : problem->getTasks())
                {
                    // Checking size
                    int cv = problem->getSize(t->getControlVariable());
                    if(t->cols() != cv)
                    {
                        throw std::runtime_error(utils::Formatter() << "Size of task " << t->getName()
                                    << " (control var " << t->getControlVariable()
                                    << " should be " << cv << " but is " << t->cols() << ")");
                    }
                    t->update(current_time,dt);
                }
            }
        }

        void updateConstraints(double current_time, double dt)
        {
            for(auto problem : problems_)
            {
                for(auto c : problem->getConstraints())
                {
                    // Checking size
                    int cv = problem->getSize(c->getControlVariable());
                    if(c->cols() != cv)
                    {
                        throw std::runtime_error(utils::Formatter() << "Size of constraint " << c->getName()
                                    << " (control var " << c->getControlVariable()
                                    << " should be " << cv << " but is " << c->cols() << ")");
                    }
                    c->update(current_time,dt);
                }
            }
        }

        std::list< std::shared_ptr<Problem> > problems_;

        std::shared_ptr<robot::RobotDynTree> robot_;

        Eigen::VectorXd joint_torque_command_;
        Eigen::VectorXd joint_acceleration_command_;

        ResolutionStrategy resolution_strategy_;
        QPSolver::SolverType solver_type_;
        const std::string& name_;
    };
} // namespace optim
} //namespace orca
