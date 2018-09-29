//|  This file is a part of the ORCA framework.
//|
//|  Copyright 2018, Fuzzy Logic Robotics
//|  Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
//|
//|  Main contributor(s): Antoine Hoarau, Ryan Lober, and
//|  Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//|
//|  ORCA is a whole-body reactive controller framework for robotics.
//|
//|  This software is governed by the CeCILL-C license under French law and
//|  abiding by the rules of distribution of free software.  You can  use,
//|  modify and/ or redistribute the software under the terms of the CeCILL-C
//|  license as circulated by CEA, CNRS and INRIA at the following URL
//|  "http://www.cecill.info".
//|
//|  As a counterpart to the access to the source code and  rights to copy,
//|  modify and redistribute granted by the license, users are provided only
//|  with a limited warranty  and the software's author,  the holder of the
//|  economic rights,  and the successive licensors  have only  limited
//|  liability.
//|
//|  In this respect, the user's attention is drawn to the risks associated
//|  with loading,  using,  modifying and/or developing or reproducing the
//|  software by the user in light of its specific status of free software,
//|  that may mean  that it is complicated to manipulate,  and  that  also
//|  therefore means  that it is reserved for developers  and  experienced
//|  professionals having in-depth computer knowledge. Users are therefore
//|  encouraged to load and test the software's suitability as regards their
//|  requirements in conditions enabling the security of their systems and/or
//|  data to be ensured and,  more generally, to use and operate it in the
//|  same conditions as regards security.
//|
//|  The fact that you are presently reading this means that you have had
//|  knowledge of the CeCILL-C license and that you accept its terms.

#pragma once
#include "orca/utils/Utils.h"
#include "orca/common/Mutex.h"
#include "orca/utils/Logger.h"
#include "orca/task/GenericTask.h"
#include "orca/optim/ControlVariable.h"
#include "orca/optim/Problem.h"
#include "orca/optim/ResolutionStrategy.h"
#include "orca/optim/QPSolver.h"
#include "orca/robot/RobotModel.h"
#include "orca/constraint/GenericConstraint.h"
#include "orca/task/RegularisationTask.h"
#include "orca/common/ConfigurableOrcaObject.h"

namespace orca
{
namespace optim
{
class Controller : public common::ConfigurableOrcaObject
{
public:
    Controller(const std::string& name);
    Controller(const std::string& name
        , std::shared_ptr<robot::RobotModel> robot
        ,ResolutionStrategy resolution_strategy
        ,QPSolverImplType solver_type);
    
    void print() const;

    void setPrintLevel(int level);

    std::shared_ptr<robot::RobotModel> robot();

    void setRobotModel(std::shared_ptr<robot::RobotModel> robot);

    bool update(double current_time, double dt);

    common::ReturnCode getReturnCode() const;

    bool addTask(std::shared_ptr<task::GenericTask> task);

    bool addTaskFromString(const std::string& task_description);
    
    bool addConstraintFromString(const std::string& task_description);
    
    template<class T>
    std::shared_ptr<T> addTask(const std::string& name)
    {
        auto t = std::make_shared<T>(name);
        if(this->addTask(t))
            return t;
        return nullptr;
    }

    std::shared_ptr<task::GenericTask> getTask(const std::string& name, int level = 0);
    std::shared_ptr<task::GenericTask> getTask(unsigned int index, int level = 0);

    bool addConstraint(std::shared_ptr<constraint::GenericConstraint> cstr);

    template<class C>
    std::shared_ptr<C> addConstraint(const std::string& name)
    {
        auto c = std::make_shared<C>(name);
        if(this->addConstraint(c))
            return c;
        return nullptr;
    }

    bool solutionFound() const;

    const Eigen::VectorXd& getSolution();

    const Eigen::VectorXd& getJointTorqueCommand(bool remove_gravity_torques = false
                                            , bool remove_coriolis_torques = false);

    const Eigen::VectorXd& computeKKTTorques();

    const Eigen::VectorXd& getJointAccelerationCommand();

    void activateTasksAndConstraints();
    void activateTasks();
    void activateConstraints();

    void deactivateTasksAndConstraints();
    void deactivateTasks();
    void deactivateConstraints();

    bool tasksAndConstraintsDeactivated();

    std::shared_ptr<task::RegularisationTask<ControlVariable::X> > globalRegularization(int level = 0);
    void setUpdateCallback(std::function<void(double,double)> update_cb);

    void removeGravityTorquesFromSolution(bool do_remove);
    void removeCoriolisTorquesFromSolution(bool do_remove);
    /**
    * @brief The recursive mutex to protect the #update function
    *
    */
    mutable common::MutexRecursive mutex;
    
    ResolutionStrategy getResolutionStrategy() const;
private:
    bool isProblemDry(std::shared_ptr<const optim::Problem> problem);
    std::shared_ptr<Problem> getProblemAtLevel(int level);
    void insertNewLevel();
    void updateTasks(double current_time, double dt);
    void updateConstraints(double current_time, double dt);
private:
    common::Parameter<ResolutionStrategy> resolution_strategy_ = ResolutionStrategy::OneLevelWeighted;
    common::Parameter<QPSolverImplType> solver_type_ = QPSolverImplType::qpOASES;
    common::Parameter<bool> remove_gravity_torques_ = false;
    common::Parameter<bool> remove_coriolis_torques_ = true;
    common::Parameter<robot::RobotModel::Ptr> robot_;
    common::Parameter<std::list< task::GenericTask::Ptr > > tasks_;
    common::Parameter<std::list< constraint::GenericConstraint::Ptr > >constraints_;
private:
    std::function<void(double,double)> update_cb_;

    std::list< Problem::Ptr > problems_;

    Eigen::VectorXd joint_torque_command_;
    Eigen::VectorXd kkt_torques_;
    Eigen::VectorXd joint_acceleration_command_;
    Eigen::VectorXd __fix_warnings__;
    
    bool solution_found_ = false;
};
} // namespace optim
} //namespace orca
