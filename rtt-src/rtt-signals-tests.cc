#include <rtt/Service.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/OperationCaller.hpp>

#include <rtt/plugin/ServicePlugin.hpp>

class ResetService : public RTT::Service {
private:
  RTT::Operation<void()> reset_op_;

public:
  ResetService(RTT::TaskContext* owner)
    : RTT::Service("reset", owner),
      reset_op_("signal_connected_peers")
  {
    this->addOperation("connectPeer", &ResetService::connectPeer, this, RTT::ClientThread);
    this->addEventOperation(reset_op_); // or getOwner()->...

    // If connected operations shall be executed the owner's thread instead of being sent,
    // uncomment the following line and replace ::send by ::call below.
    //reset_op_.getOperationCaller()->setThread(RTT::OwnThread, owner->engine());
  }

  bool connectPeer(const std::string &peer_name, const std::string &op_name)
  {
    RTT::TaskContext *peer = getOwner()->getPeer(peer_name);
    if (!peer) return false;
    RTT::OperationCaller<void()> peer_reset(peer->getOperation(op_name), this->getOwner()->engine());
    if (!peer_reset.ready()) return false;

    RTT::Handle handle = reset_op_.signals(boost::bind(&RTT::OperationCaller<void()>::send, peer_reset));
    return handle.connected();
  }
};

ORO_SERVICE_NAMED_PLUGIN(ResetService, "reset_service")