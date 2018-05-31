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

#include "orca/robot/RobotDynTree.h"
#include "orca/optim/ControlVariable.h"
#include "orca/optim/Problem.h"
#include "orca/common/Wrench.h"

namespace orca
{

namespace common
{
    class TaskBase
    {
        friend optim::Problem;
    public:
        enum State {Init,Resized,Deactivated,Activating,Activated,Deactivating,Error};

        TaskBase(const std::string& name, optim::ControlVariable control_var);
        virtual ~TaskBase();
        bool isActivated() const;
        void setRobotModel(std::shared_ptr<robot::RobotDynTree> robot);

        optim::ControlVariable getControlVariable() const;

        const std::string& getName() const;

        virtual bool activate();
        virtual void update(double current_time, double dt);
        virtual bool deactivate();
        virtual void print() const;
        /**
        * @brief Check if the constraint is inserted in the problem
        *
        * @return bool
        */
        virtual bool setProblem(std::shared_ptr< const orca::optim::Problem > problem);

        bool hasProblem() const;
        bool hasRobot() const;
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
        std::shared_ptr<const robot::RobotDynTree> getRobot() const;

        void link(std::shared_ptr<common::TaskBase> e);
        void setUpdateCallback(std::function<void(double,double)> update_cb);
        void setActivationCallback(std::function<void(void)> activation_cb);
        void setDeactivationCallback(std::function<void(void)> deactivation_cb);
    protected:
        virtual void resize();
        std::shared_ptr<robot::RobotDynTree> robot();
        std::shared_ptr<common::Wrench> wrench();

        virtual void onResize() = 0;
        virtual void onActivation() = 0;
        virtual bool rampUp(double time_since_start);
        void setRampValue(double new_val);
        virtual void onUpdate(double current_time, double dt) = 0;
        virtual bool rampDown(double time_since_stop);
        virtual void onDeactivation() = 0;
    private:
        void checkIfUpdatable() const;
        bool is_activated_ = true;
        State state_ = Init;
        double start_time_ = 0;
        double stop_time_ = 0;
        double ramp_duration_ = 0;
        double ramp_value_ = 0;
        bool activation_requested_ = false;
        bool deactivation_requested_ = false;
        const std::string name_;
        std::shared_ptr<const optim::Problem> problem_;
        std::shared_ptr<robot::RobotDynTree> robot_;
        std::shared_ptr<common::Wrench> wrench_;
        optim::ControlVariable control_var_;
        std::list<std::shared_ptr<common::TaskBase> > linked_elements_;
        std::function<void(double,double)> update_cb_;
        std::function<void(void)> activation_cb_;
        std::function<void(void)> deactivation_cb_;
        //unsigned int getHierarchicalLevel() const;
        //void getHierarchicalLevel(unsigned int level);
        //unsigned int hierarchical_level = 0;
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
