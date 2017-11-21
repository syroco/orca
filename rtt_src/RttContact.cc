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
    class RttContact: public orca::constraint::Contact, public RTT::TaskContext
    {
    public:
        RttContact(const std::string& name)
        : RTT::TaskContext(name)
        , robotHelper_(this,this,this->robot())
        {
            orca::constraint::Contact::setName(name);
            this->addOperation("insertInProblem",&orca::constraint::Contact::insertInProblem,this,RTT::OwnThread);
            this->addOperation("removeFromProblem",&orca::constraint::Contact::removeFromProblem,this,RTT::OwnThread);
            this->addOperation("desactivate",&orca::constraint::Contact::desactivate,this,RTT::OwnThread);
            this->addOperation("activate",&orca::constraint::Contact::activate,this,RTT::OwnThread);
            
            this->addOperation("setContactFrame",&orca::constraint::Contact::setContactFrame,this,RTT::OwnThread);
            this->addOperation("getFrictionCoeff",&orca::constraint::Contact::getFrictionCoeff,this,RTT::OwnThread);
            this->addOperation("getMargin",&orca::constraint::Contact::getMargin,this,RTT::OwnThread);
            this->addOperation("setAngleOffset",&orca::constraint::Contact::setAngleOffset,this,RTT::OwnThread);
            this->addOperation("setFrictionCoeff",&orca::constraint::Contact::setFrictionCoeff,this,RTT::OwnThread);
            this->addOperation("setMargin",&orca::constraint::Contact::setMargin,this,RTT::OwnThread);
            this->addOperation("setConeOrientation",&orca::constraint::Contact::setConeOrientation,this,RTT::OwnThread);
            this->addOperation("getConeOrientation",&orca::constraint::Contact::getConeOrientation,this,RTT::OwnThread);
            this->addOperation("setNumberOfFaces",&orca::constraint::Contact::setNumberOfFaces,this,RTT::OwnThread);
            this->addOperation("getJacobianTranspose",&orca::constraint::Contact::getJacobianTranspose,this,RTT::OwnThread);
            this->addOperation("setContactFrame",&orca::constraint::Contact::setContactFrame,this,RTT::OwnThread);
        }

        bool configureHook()
        {
            robotHelper_.configureRobotPorts();
            return true;
        }

        void updateHook()
        {
            robotHelper_.updateRobotModel();
            orca::constraint::Contact::update();
        }
    private:
        robot::RobotModelHelper robotHelper_;
    };

}
}
ORO_CREATE_COMPONENT(rtt_orca::constraint::RttContact)
