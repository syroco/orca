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
#include <orca/rtt/rtt-RobotModelHelper.h>

namespace rttorca
{
    using namespace RTT;

    class DynEq: public RTT::TaskContext
    {
    public:
        DynEq (const std::string& name)
        : RTT::TaskContext(name)
        , checkConn_("checkContactsConnections")
        , robotHelper_(this,dyn_eq_,dyn_eq_.robot())
        {
            dyn_eq_.setName(name);
            
            this->addOperation("checkContactsConnections",&DynEq::checkContactsConnections,this,RTT::OwnThread);
            checkConn_ = this->getOperation("checkContactsConnections");
        }

        bool configureHook()
        {
            int ndof = dyn_eq_.robot().getNrOfDegreesOfFreedom();
            Jt_.resize(6,6+ndof);
            return ndof > 0;
        }

        void updateHook()
        {
            robotHelper_.updateModel();
            checkConn_.call();
            if(checkConn_.ready())
            {
                log(Info) << "Conn ready"  << endlog();
                int ncontacts = contacts_map_.size();

                log(Info) << "We have " << ncontacts  << endlog();

                bool use_sync = true;
                if(use_sync)
                {
                    int i=0;
                    for(auto c : contacts_map_)
                    {
                        log(Info) << "Collecting J from "<< c.first << endlog();
                        //dyn_eq_.setJacobianTransposeBlock(i,c.second.func_.call());
                        i++;

                    }
                }
                else
                {
                    for(auto c : contacts_map_)
                    {
                        c.second.handle_ = c.second.func_.send();
                    }

                    int i=0;
                    for(auto c : contacts_map_)
                    {
                        RTT::SendStatus s;
                        if( (s = c.second.handle_.collectIfDone(Jt_)) == RTT::SendSuccess)
                        {
                            log(Info) << "Collecting J from "<< c.first << endlog();
                            //dyn_eq_.setJacobianTransposeBlock(i,Jt_);
                        }
                        else
                        {
                            log(Info) << s << " Not ready <--" << c.first << endlog();
                        }
                        i++;
                    }
                }

            }

            dyn_eq_.update();
        }

        void checkContactsConnections()
        {
            for(auto p : getPeerList())
            {
                if(contacts_map_.find(p) == contacts_map_.end())
                {
                    auto peer = getPeer(p);
                    if(peer->provides("state") && peer->provides("state")->hasOperation("getJacobianTranspose"))
                    {
                        if(peer->isConfigured() && peer->isRunning())
                        {
                            log(Info) << "Adding new client " << p << endlog();
                            contacts_map_[p] = ClientConnection(peer->provides("state")->getOperation("getJacobianTranspose"));
                            contacts_map_[p].func_.setCaller(this->engine());
                            log(Info) << "New size is now " << contacts_map_.size()  << endlog();
                        }
                    }
                }
            }

            auto it = std::begin(contacts_map_);
            while(it != std::end(contacts_map_))
            {
                if(!it->second.func_.ready())
                {
                    log(Warning) << "Removing broken connection with client "<<it->first<<endlog();
                    it = contacts_map_.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            return;
        }
    protected:
        orca::constraint::DynamicsEquationConstraint dyn_eq_;
            // Useful for threaded updates
        struct ClientConnection
        {
            ClientConnection(){}
            ClientConnection(RTT::OperationCaller<const Eigen::MatrixXd&(void)> f)
            : func_(f)
            {

            }
            RTT::OperationCaller<const Eigen::MatrixXd&(void)> func_;
            RTT::SendHandle<const Eigen::MatrixXd&(void)> handle_;
        };

        std::map<std::string,ClientConnection> contacts_map_;
        RTT::OperationCaller<void(void)> checkConn_;
        Eigen::MatrixXd Jt_;
        RobotModelHelper robotHelper_;
    };

}

ORO_CREATE_COMPONENT(rttorca::DynEq)
