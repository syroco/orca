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

#include "orca/robot/RobotModel.h"
#include "orca/optim/ControlVariable.h"
#include "orca/optim/Problem.h"
#include "orca/common/Mutex.h"
#include "orca/common/Parameter.h"

namespace orca
{
namespace common
{
    class Wrench;
    /**
    * @brief The common base class for tasks and constraints
    *
    * This class contains a model of the robot, the problem in which the tasks
    * is currently being used, and a state machine. Although this class is
    * called TaskBase, both tasks and constraints inherit from this.
    */
    class TaskBase
    {
        friend optim::Problem;
    public:
        using Ptr = std::shared_ptr<TaskBase>;
        /**
        * @brief Represents the internal state of the task
        */
        enum State {
             Init = 0/*!< Task is instanciated */
            ,Resized /*!< The robot and the problem have been set */
            ,Activating /*!< Task is running but ramping up */
            ,Activated /*!< Task is running */
            ,Deactivating /*!< Task is running but ramping down */
            ,Deactivated /*!< Task has finishing ramping down and is now stopped */
            ,Error /*!< Task update returned an error (not used yet) */
        };
        /**
        * @brief Configure the task from YAML/JSON string. It must contain all the required parameters.
        *
        * @return true is all the required parameters are loaded properly
        */
        bool configureFromString(const std::string& yaml_str);
        /**
        * @brief Configure the task from YAML/JSON file. It must contain all the required parameters.
        *
        * @return true is all the required parameters are loaded properly
        */
        bool configureFromFile(const std::string& yaml_url);
        /**
        * @brief Returns true if all params added with @addParameter have been set
        *
        * @return true is all the required parameters are loaded properly
        */
        bool isConfigured() const;
        
        enum ParamPolicy
        {
              Required = 0
            , Optional
        };
        void addParameter(const std::string& param_name,ParameterBase* param
                    , ParamPolicy policy = Required
                    , std::function<void()> on_loading_success = 0
                    , std::function<void()> on_loading_failed = 0);
        
        ParameterBase* getParameter(const std::string& param_name);
        void printParameters() const;
        
        TaskBase(const std::string& name, optim::ControlVariable control_var);
        virtual ~TaskBase();
        bool isActivated() const;
        bool isComputing() const;
        void setRobotModel(std::shared_ptr<robot::RobotModel> robot);

        optim::ControlVariable getControlVariable() const;

        const std::string& getName() const;
        void setName(const std::string& name);

        virtual bool activate();
        virtual void update(double current_time, double dt);
        virtual bool deactivate();
        virtual void print() const;

        virtual bool setProblem(std::shared_ptr< const orca::optim::Problem > problem);

        bool hasProblem() const;
        bool hasRobot() const;
        bool dependsOnProblem() const;
        bool dependsOnRobotJoints() const;
        bool dependsOnFloatingBase() const;
        bool hasWrench() const;
        bool isRobotInitialized() const;
        State getState() const;
        void setRampDuration(double ramp_time);
        double getRampDuration() const;
        double getCurrentRampValue() const;

        double getStartTime() const;
        double getStopTime() const;

        std::shared_ptr<const optim::Problem> getProblem()const;
        std::shared_ptr<const common::Wrench> getWrench() const;
        std::shared_ptr<const robot::RobotModel> getRobot() const;

        
        /**
        * @brief Add a child/slave task that will be updated BEFORE the parent task
        */
        void addChild(std::shared_ptr<TaskBase> e);
        /**
        * @brief Returns true if the task owned by a parent task.
        */
        bool hasParent() const;
        /**
        * @brief Returns true if the task has children
        */
        bool hasChildren() const;

        void onResizedCallback(std::function<void(void)> cb);
        void onActivationCallback(std::function<void(void)> cb);
        void onActivatedCallback(std::function<void(void)> cb);
        void onComputeBeginCallback(std::function<void(double,double)> cb);
        void onComputeEndCallback(std::function<void(double,double)> cb);
        void onDeactivationCallback(std::function<void(void)> cb);
        void onDeactivatedCallback(std::function<void(void)> cb);
        /**
        * @brief The recursive mutex that is locked during the #update function.
        * It is up to the external user to lock this mutex to protect the task attributes.
        */
        mutable orca::common::MutexRecursive mutex;
    protected:
        virtual void resize();
        std::shared_ptr<robot::RobotModel> robot();
        std::shared_ptr<Wrench> wrench();

        virtual void onResize() = 0;
        virtual void onResized() {};
        virtual void onActivation() {};
        virtual void onActivated() {};
        virtual bool rampUp(double time_since_start);
        void setRampValue(double new_val);
        virtual void onCompute(double current_time, double dt) = 0;
        virtual bool rampDown(double time_since_stop);
        virtual void onDeactivation() {};
        virtual void onDeactivated() {};
    private:
        void setParentName(const std::string& parent_name);
        void checkIfUpdatable() const;
        bool is_activated_ = true;
        State state_ = Init;
        double start_time_ = 0;
        double stop_time_ = 0;
        double ramp_duration_ = 0;
        double ramp_value_ = 0;
        bool activation_requested_ = false;
        bool deactivation_requested_ = false;
        std::string name_;
        std::shared_ptr<const optim::Problem> problem_;
        std::shared_ptr<robot::RobotModel> robot_;
        std::shared_ptr<Wrench> wrench_;
        optim::ControlVariable control_var_;
        std::list<std::shared_ptr<TaskBase> > children_;

        std::function<void(void)> on_resized_cb_;
        std::function<void(void)> on_activation_cb_;
        std::function<void(void)> on_activated_cb_;
        std::function<void(double,double)> on_update_begin_cb_;
        std::function<void(double,double)> on_update_end_cb_;
        std::function<void(void)> on_deactivation_cb_;
        std::function<void(void)> on_deactivated_cb_;
        //unsigned int getHierarchicalLevel() const;
        //void getHierarchicalLevel(unsigned int level);
        //unsigned int hierarchical_level = 0;
        std::map<std::string, ParameterBase* > parameters_;
        std::string parent_name_;
        double current_time_ = -1;
        double current_dt_ = 0;
    };


    inline ::std::ostream& operator<<(::std::ostream& os, const TaskBase::State& s)
    {
        switch (s)
        {
            case TaskBase::Init: os << "Init"; break;
            case TaskBase::Resized: os << "Resized"; break;
            case TaskBase::Activating: os << "Activating"; break;
            case TaskBase::Activated: os << "Activated"; break;
            case TaskBase::Deactivating: os << "Deactivating"; break;
            case TaskBase::Deactivated: os << "Deactivated"; break;
            case TaskBase::Error: os << "Error"; break;
            default: break;
        }
        return os;
    }
} // namespace common
} // namespace orca

// This header needs to have TaskBase
// TODO: figure out if forward declaring is possible in that case
#include "orca/common/ParameterSharedPtr.h"
#include "orca/common/Wrench.h"
