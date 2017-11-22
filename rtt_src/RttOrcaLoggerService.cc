#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Property.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>


#include <orca/util/Logger.h>

namespace rtt_orca
{
namespace util
{
    struct RttOrcaLoggerService: public RTT::Service
    {
        RttOrcaLoggerService(RTT::TaskContext* owner)
        : RTT::Service("orca_logger",owner)
        {
            this->addConstant("none",orca::util::LogLevel::none);
            this->addConstant("fatal",orca::util::LogLevel::fatal);
            this->addConstant("error",orca::util::LogLevel::error);
            this->addConstant("warning",orca::util::LogLevel::warning);
            this->addConstant("info",orca::util::LogLevel::info);
            this->addConstant("debug",orca::util::LogLevel::debug);
            this->addConstant("verbose",orca::util::LogLevel::verbose);
            this->addOperation("setLogLevel", &RttOrcaLoggerService::setLogLevel, this);
            this->setLogLevel(orca::util::LogLevel::debug);
        }
        
        void setLogLevel(orca::util::LogLevel level)
        {
            orca::util::Logger::setLogLevel( level  );
        }
    };
}
}


ORO_SERVICE_NAMED_PLUGIN(rtt_orca::util::RttOrcaLoggerService,"orca_logger")
