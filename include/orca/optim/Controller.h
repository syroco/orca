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
        Controller(std::shared_ptr<robot::RobotDynTree> robot
            ,ResolutionStrategy resolution_strategy
            ,QPSolver::SolverType solver_type)
        : robot_(robot)
        , resolution_strategy_(resolution_strategy)
        {
            if(resolution_strategy != ResolutionStrategy::OneLevelWeighted)
            {
                throw std::runtime_error(utils::Formatter() << "Only ResolutionStrategy::OneLevelWeighted is supported for now");
            }
            // Push at least One Level
            problem_ = std::make_shared<Problem>(solver_type);
        }
        std::shared_ptr<robot::RobotDynTree> robot()
        {
            return robot_;
        }

        void setRobotModel(std::shared_ptr<robot::RobotDynTree> robot)
        {
            robot_ = robot;
            resizeAll();
        }

        void update(double current_time, double dt)
        {
            if(resolution_strategy_ == ResolutionStrategy::OneLevelWeighted)
            {
                updateTasks(current_time,dt);
                updateConstraints(current_time,dt);
                problem_->build();
                problem_->print();
                problem_->solve();
            }
        }
        
        bool addTask(std::shared_ptr<task::GenericTask> task)
        {
            if(!exists(task,tasks_))
            {
                task->setRobotModel(robot_);
                task->setProblem(problem_);
                LOG_INFO << "Adding task " << task->getName();
                tasks_.push_back(task);
                resizeAll();
                return true;
            }
            else
            {
                LOG_WARNING << "Task " << task->getName() << " is already present in the controller";
                return false;
            }
        }
        
        bool addTask(std::shared_ptr<task::WrenchTask> task)
        {
            addWrench(task->getWrench());
            addTask(task);
        }
        
        bool addConstraint(std::shared_ptr<constraint::GenericConstraint> cstr)
        {
            if(!exists(cstr,constraints_))
            {
                cstr->setRobotModel(robot_);
                cstr->setProblem(problem_);
                LOG_INFO << "Adding constraint " << cstr->getName();
                constraints_.push_back(cstr);
                resizeAll();
                return true;
            }
            else
            {
                LOG_WARNING << "Constraint " << cstr->getName() << " is already present in the controller";
                return false;
            }
        }

        const Eigen::VectorXd& getJointTorqueCommand()
        {

        }
        const Eigen::VectorXd& getJointAccelerationCommand()
        {

        }
    protected:
        bool addWrench(std::shared_ptr<const common::Wrench> wrench)
        {
            if(!exists(wrench,wrenches_))
            {
                LOG_INFO << "Adding Wrench " << wrench->getName();
                wrenches_.push_back(wrench);
                resizeAll();
                return true;
            }
            else
            {
                LOG_WARNING << "Wrench " << wrench->getName() << " is already present in the controller";
                return false;
            }
        }
        
        void resizeAll()
        {
            resizeProblem();
            resizeTasks();
            resizeConstraints();
        }
        
        void resizeProblem()
        {
            problem_->setProblemObjects(robot_->getNrOfDegreesOfFreedom(),tasks_,constraints_,wrenches_);
        }
        
        void resizeTasks()
        {
            for(auto task : tasks_)
            {
                LOG_DEBUG << "Resizing task " << task->getName();
                task->resize();
            }
        }

        void resizeConstraints()
        {
            for(auto constr : constraints_)
            {
                LOG_DEBUG << "Resizing constraint " << constr->getName();
                constr->resize();
            }
        }
        
        void updateTasks(double current_time, double dt)
        {
            for(auto t : tasks_)
            {
                // Checking size
                int cv = problem_->getSize(t->getControlVariable());
                if(t->cols() != cv)
                {
                    throw std::runtime_error(utils::Formatter() << "Size of task " << t->getName()
                                << " (control var " << t->getControlVariable()
                                << " should be " << cv << " but is " << t->cols());
                }
                t->update(current_time,dt);
            }
        }
        
        void updateConstraints(double current_time, double dt)
        {
            for(auto c : constraints_)
            {
                // Checking size
                int cv = problem_->getSize(c->getControlVariable());
                if(c->cols() != cv)
                {
                    throw std::runtime_error(utils::Formatter() << "Size of constraint " << c->getName()
                                << " (control var " << c->getControlVariable()
                                << " should be " << cv << " but is " << c->cols());
                }
                c->update(current_time,dt);
            }
        }       
        void updateWrenches()
        {
            for(auto w : wrenches_)
            {
                // Checking size
                int cv = problem_->getSize(w->getControlVariable());
                int cols = w->getJacobian().cols();
                if(cols != cv)
                {
                    throw std::runtime_error(utils::Formatter() << "Size of Wrench " << w->getName()
                                << " (control var " << w->getControlVariable()
                                << " should be " << cv << " but is " << cols);
                }
            }
        }          
        template<class T> bool exists(const std::shared_ptr<T> t, std::list< std::shared_ptr<T> > l){
            return std::find(l.begin(),l.end(),t) != l.end();
        }
        std::list< std::shared_ptr<task::GenericTask> > tasks_;
        std::list< std::shared_ptr<constraint::GenericConstraint> > constraints_;
        std::list< std::shared_ptr<const common::Wrench> > wrenches_;

        std::shared_ptr<optim::Problem> problem_;
        std::shared_ptr<robot::RobotDynTree> robot_;

        Eigen::VectorXd joint_torque_command_;
        Eigen::VectorXd joint_acceleration_command_;
        
        ResolutionStrategy resolution_strategy_;
    };
} // namespace optim
} //namespace orca
