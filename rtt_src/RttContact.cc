#include <orca/rtt_orca/common/RttTaskCommon.h>

namespace rtt_orca
{
namespace constraint
{
    class RttContact: public orca::constraint::Contact, public common::RttTaskCommon
    {
    public:
        RttContact(const std::string& name)
        : common::RttTaskCommon(this,this,name)
        {
            this->addOperation("insertInProblem",&orca::constraint::Contact::insertInProblem,this,RTT::OwnThread);
            this->addOperation("removeFromProblem",&orca::constraint::Contact::removeFromProblem,this,RTT::OwnThread);
            this->addOperation("desactivate",&orca::constraint::Contact::desactivate,this,RTT::OwnThread);
            this->addOperation("activate",&orca::constraint::Contact::activate,this,RTT::OwnThread);
            
            this->addOperation("setControlFrame",&orca::constraint::Contact::setControlFrame,this,RTT::OwnThread);
            this->addOperation("getFrictionCoeff",&orca::constraint::Contact::getFrictionCoeff,this,RTT::OwnThread);
            this->addOperation("getMargin",&orca::constraint::Contact::getMargin,this,RTT::OwnThread);
            this->addOperation("setAngleOffset",&orca::constraint::Contact::setAngleOffset,this,RTT::OwnThread);
            this->addOperation("setFrictionCoeff",&orca::constraint::Contact::setFrictionCoeff,this,RTT::OwnThread);
            this->addOperation("setMargin",&orca::constraint::Contact::setMargin,this,RTT::OwnThread);
            this->addOperation("setConeOrientation",&orca::constraint::Contact::setConeOrientation,this,RTT::OwnThread);
            this->addOperation("getConeOrientation",&orca::constraint::Contact::getConeOrientation,this,RTT::OwnThread);
            this->addOperation("setNumberOfFaces",&orca::constraint::Contact::setNumberOfFaces,this,RTT::OwnThread);
            this->addOperation("setControlFrame",&orca::constraint::Contact::setControlFrame,this,RTT::OwnThread);
        }

        void updateHook()
        {
            if(this->updateRobotModel())
            {
                orca::constraint::Contact::update();
            }
        }
    };

}
}
ORO_CREATE_COMPONENT(rtt_orca::constraint::RttContact)
