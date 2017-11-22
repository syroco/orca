#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Property.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <type_traits>
#include <orca/orca.h>

namespace rtt_orca
{
namespace util
{
    class RttOrcaRegister: public RTT::Service
    {
    public:
        RttOrcaRegister(RTT::TaskContext* owner)
        : RTT::Service("orca_register",owner)
        , resize_op_("resize_connected_peers")
        {
            this->addOperation("connectPeer", &RttOrcaRegister::connectPeer, this, RTT::ClientThread);
            this->addEventOperation(resize_op_); // or getOwner()->...

            // If connected operations shall be executed the owner's thread instead of being sent,
            // uncomment the following line and replace ::send by ::call below.
            //resize_op_.getOperationCaller()->setThread(RTT::OwnThread, owner->engine());
        }
        
        bool connectPeer(const std::string &peer_name, const std::string &op_name)
        {
          RTT::TaskContext *peer = getOwner()->getPeer(peer_name);
          if (!peer) return false;
          RTT::OperationCaller<void()> peer_resiwe(peer->getOperation(op_name), this->getOwner()->engine());
          if (!peer_resiwe.ready()) return false;

          RTT::Handle handle = resize_op_.signals(boost::bind(&RTT::OperationCaller<void()>::send, peer_resiwe));
          return handle.connected();
        }
        
    private:
      RTT::Operation<void()> resize_op_;
    };
}
}


ORO_SERVICE_NAMED_PLUGIN(rtt_orca::util::RttOrcaRegister,"orca_register")
