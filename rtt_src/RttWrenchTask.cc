#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Operation.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Property.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include <orca/orca.h>
#include <orca/rtt_orca/robot/RobotModelHelper.h>

namespace rtt_orca
{
namespace task
{
    class RttWrenchTask: public orca::task::WrenchTask, public RTT::TaskContext
    {
    public:
        RttWrenchTask(const std::string& name)
        : RTT::TaskContext(name)
        , robotHelper_(this,this,this->robot())
        {
            orca::task::WrenchTask::setName(name);
            this->addOperation("insertInProblem",&orca::task::GenericTask::insertInProblem,this,RTT::OwnThread);
            this->addOperation("removeFromProblem",&orca::task::GenericTask::removeFromProblem,this,RTT::OwnThread);
            
            this->addOperation("setBaseFrame",&orca::task::WrenchTask::setBaseFrame,this,RTT::OwnThread);
            this->addOperation("setControlFrame",&orca::task::WrenchTask::setControlFrame,this,RTT::OwnThread);
            this->addOperation("getBaseFrame",&orca::task::WrenchTask::getBaseFrame,this,RTT::OwnThread);
            this->addOperation("getControlFrame",&orca::task::WrenchTask::getControlFrame,this,RTT::OwnThread);
            
            this->provides("pid")->addOperation("setProportionalGain",&orca::common::PIDController<6>::setProportionalGain,&this->pid(),RTT::OwnThread);
            this->provides("pid")->addOperation("setDerivativeGain",&orca::common::PIDController<6>::setDerivativeGain,&this->pid(),RTT::OwnThread);
            
            this->addOperation("setCurrent",&orca::task::WrenchTask::setCurrent,this,RTT::OwnThread);
            this->addOperation("setDesired",&orca::task::WrenchTask::setDesired,this,RTT::OwnThread);
        }

        bool configureHook()
        {
            robotHelper_.configureRobotPorts();
            return true;
        }

        void updateHook()
        {
            robotHelper_.updateRobotModel();
            port_wrench_des_.readNewest(this->wrench_des_);
            port_current_wrench_.readNewest(this->current_wrench_);
            orca::task::WrenchTask::update();
        }

    protected:
        robot::RobotModelHelper robotHelper_;
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_wrench_des_;
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_current_wrench_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::task::RttWrenchTask)
