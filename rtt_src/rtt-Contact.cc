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
#include <rtt/os/Mutex.hpp>
#include <orca/orca.h>
#include <orca/rtt_orca/RobotModelHelper.h>

namespace rtt_orca
{
namespace constraint
{
    class RttContact: public orca::constraint::Contact, public RTT::TaskContext
    {
    public:
        RttContact(const std::string& name)
        : RTT::TaskContext(name)
        , robotHelper_(this,contact_,contact_.robot())
        {
            orca::constraint::Contact::setName(name);
            this->addOperation("setContactFrame",&orca::constraint::Contact::setContactFrame,&this,RTT::OwnThread);
        }

        bool configureHook()
        {
            robotHelper_.configureRobotPorts();
            return true;
        }

        void updateHook()
        {
            contact_.update();
        }
    private:
        RobotModelHelper robotHelper_;
    };

}

ORO_CREATE_COMPONENT(rttorca::constraint::RttContact)
