#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Time.hpp>

struct SomeClassWithReset : public RTT::TaskContext
{
    SomeClassWithReset(const std::string& name)
    : RTT::TaskContext(name)
    {
        this->addOperation("reset",&SomeClassWithReset::reset,this,RTT::OwnThread);
    }
    
    void reset()
    {
        RTT::log(RTT::Info) << getName() << "-->reset() started at " << RTT::os::TimeService::Instance()->getTicks() << RTT::endlog();
        usleep(1E6);
        RTT::log(RTT::Info) << getName() << "-->reset() ended   at " << RTT::os::TimeService::Instance()->getTicks() << RTT::endlog();
    }
    void updateHook()
    {
        RTT::log(RTT::Info) << getName() << "-->updateHook() at " << RTT::os::TimeService::Instance()->getTicks() << RTT::endlog();
    }
};


ORO_CREATE_COMPONENT(SomeClassWithReset)
