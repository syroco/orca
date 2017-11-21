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
    template<orca::optim::ControlVariable C>
    class RttRegularisationTask: public orca::task::RegularisationTask<C>, public RTT::TaskContext
    {
    public:
        RttRegularisationTask(const std::string& name)
        : RTT::TaskContext(name)
        , robotHelper_(this,this,this->robot())
        {
            orca::task::RegularisationTask<C>::setName(name);
            this->addOperation("insertInProblem",&orca::task::GenericTask::insertInProblem,this,RTT::OwnThread);
            this->addOperation("removeFromProblem",&orca::task::GenericTask::removeFromProblem,this,RTT::OwnThread);
        }

        bool configureHook()
        {
            robotHelper_.configureRobotPorts();
            return true;
        }

        void updateHook()
        {
            robotHelper_.updateRobotModel();
            orca::task::RegularisationTask<C>::update();
        }

    protected:
        robot::RobotModelHelper robotHelper_;
    };
}
}

ORO_LIST_COMPONENT_TYPE(rtt_orca::task::RttRegularisationTask<orca::optim::ControlVariable::X>)
ORO_LIST_COMPONENT_TYPE(rtt_orca::task::RttRegularisationTask<orca::optim::ControlVariable::GeneralisedAcceleration>)
ORO_LIST_COMPONENT_TYPE(rtt_orca::task::RttRegularisationTask<orca::optim::ControlVariable::GeneralisedTorque>)
ORO_LIST_COMPONENT_TYPE(rtt_orca::task::RttRegularisationTask<orca::optim::ControlVariable::ExternalWrench>)
ORO_CREATE_COMPONENT_LIBRARY()
