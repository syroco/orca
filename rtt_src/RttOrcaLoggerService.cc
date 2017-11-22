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
            this->addConstant("none",static_cast<int>(orca::util::LogLevel::none));
            this->addConstant("fatal",static_cast<int>(orca::util::LogLevel::fatal));
            this->addConstant("error",static_cast<int>(orca::util::LogLevel::error));
            this->addConstant("warning",static_cast<int>(orca::util::LogLevel::warning));
            this->addConstant("info",static_cast<int>(orca::util::LogLevel::info));
            this->addConstant("debug",static_cast<int>(orca::util::LogLevel::debug));
            this->addConstant("verbose",static_cast<int>(orca::util::LogLevel::verbose));
            this->addOperation("setLogLevel", &RttOrcaLoggerService::setLogLevel, this);
            this->setLogLevel(static_cast<int>(orca::util::LogLevel::debug));
        }
        
        void setLogLevel(int level)
        {
            orca::util::Logger::setLogLevel( static_cast<orca::util::LogLevel>(level) );
        }
    };
}
}


ORO_SERVICE_NAMED_PLUGIN(rtt_orca::util::RttOrcaLoggerService,"orca_logger")
