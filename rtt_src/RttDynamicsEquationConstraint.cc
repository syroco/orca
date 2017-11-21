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
namespace constraint
{
    class RttDynamicsEquationConstraint: public orca::constraint::DynamicsEquationConstraint, public RTT::TaskContext
    {
    public:
        RttDynamicsEquationConstraint(const std::string& name)
        : RTT::TaskContext(name)
        , robotHelper_(this,this,this->robot())
        {
            orca::constraint::DynamicsEquationConstraint::setName(name);
        }

        bool configureHook()
        {
            robotHelper_.configureRobotPorts();
            return true;
        }

        void updateHook()
        {
            robotHelper_.updateRobotModel();
            orca::constraint::DynamicsEquationConstraint::update();
        }

    protected:
        robot::RobotModelHelper robotHelper_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::constraint::RttDynamicsEquationConstraint)
