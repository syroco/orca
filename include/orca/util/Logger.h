#pragma once

#include <plog/Log.h>


namespace orca
{
    namespace util
    {

        enum class LogLevel
        {
            none = 0,
            fatal = 1,
            error = 2,
            warning = 3,
            info = 4,
            debug = 5,
            verbose = 6
        };

        struct Logger
        {
            Logger();
            static void setLogLevel(LogLevel log_level);
        };
    }
}

#ifdef _WIN32
#include <orca/optim/ControlVariable.h>
namespace plog
{
    Record& operator<<(Record& record, const orca::optim::ControlVariable& t);
}
#endif
