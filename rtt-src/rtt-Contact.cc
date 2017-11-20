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
#include <orca/rtt/rtt-RobotModelHelper.h>

namespace rttorca
{
    class Contact: public RTT::TaskContext
    {
    public:
        Contact (const std::string& name)
        : RTT::TaskContext(name)
        , robotHelper_(this,contact_,contact_.robot())
        {
            contact_.setName(name);
            
            this->provides("state")->addOperation("getJacobian",&rttorca::Contact::getJacobianTranspose,this,RTT::ClientThread);
            this->addOperation("setContactFrame",&orca::constraint::Contact::setContactFrame,&contact_,RTT::OwnThread);
        }

        const Eigen::MatrixXd& getJacobianTranspose()
        {
            RTT::os::MutexLock l(mutex_);
            RTT::log(RTT::Info) << "Giving jacobian" << RTT::endlog();
            return contact_.getJacobianTranspose();
        }

        bool configureHook()
        {
            return true;
        }

        void updateHook()
        {
            robotHelper_.updateRobotModel();
            if(mutex_.trylock())
            {
                contact_.update();
                mutex_.unlock();
            }
        }
    private:
        RTT::os::Mutex mutex_;
        orca::constraint::Contact contact_;
        RobotModelHelper robotHelper_;
        std::string contactFrame_;
    };

}

ORO_CREATE_COMPONENT(rttorca::Contact)
