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
#include "orca/optim/Problem.h"

namespace orca
{

namespace common
{
    class TaskBase
    {
    public:
        enum State {Init,Resized,Starting,Running,ShuttingDown,Stopped,Error};

        TaskBase(const std::string& name,optim::ControlVariable control_var);
        virtual ~TaskBase();

        void setRobotModel(std::shared_ptr<robot::RobotDynTree> robot);

        bool loadRobotModel(const std::string& file_url);

        optim::ControlVariable getControlVariable() const;

        const std::string& getName() const;

        virtual bool start(double current_time);
        virtual void update(double current_time, double dt);
        virtual bool stop(double current_time);
        virtual void resize();
        virtual void print() const;

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
        virtual bool setProblem(std::shared_ptr< const orca::optim::Problem > problem);

        std::shared_ptr<robot::RobotDynTree> robot();
        std::shared_ptr<const optim::Problem> problem();

        bool hasProblem() const;
        bool hasRobot() const;
        bool isRobotInitialized() const;
        State getState() const;
        void setRampDuration(double ramp_time);
        double getRampDuration() const;
        double getCurrentRampValue() const;

        double getStartTime() const;
        double getStopTime() const;
    protected:
        virtual void onResize() = 0;
        virtual void onStart() = 0;
        virtual bool rampUp(double time_since_start);
        void setRampValue(double new_val);
        virtual void onUpdate(double current_time, double dt) = 0;
        virtual bool rampDown(double time_since_stop);
        virtual void onStop() = 0;
    private:
        void checkIfUpdatable() const;
        bool is_activated_ = true;
        State state_ = Init;
        double start_time_ = 0;
        double stop_time_ = 0;
        double ramp_duration_ = .5;
        double ramp_value_ = 0;
        const std::string name_;
        std::shared_ptr<const optim::Problem> problem_;
        std::shared_ptr<robot::RobotDynTree> robot_;
        optim::ControlVariable control_var_;
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
            case TaskBase::Starting: os << "Starting"; break;
            case TaskBase::Running: os << "Running"; break;
            case TaskBase::ShuttingDown: os << "ShuttingDown"; break;
            case TaskBase::Stopped: os << "Stopped"; break;
            case TaskBase::Error: os << "Error"; break;
            default: break;
        }
        return os;
    }
} // namespace common
} // namespace orca
