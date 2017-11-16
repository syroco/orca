#include <orca/util/Logger.h>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>

namespace orca
{
    namespace util
    {
        static plog::RollingFileAppender<plog::TxtFormatter> fileAppender("orca-log.txt", 8000, 3);
        static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;

        Logger::Logger()
        {
            plog::init(plog::verbose, &fileAppender).addAppender(&consoleAppender);
            //LOG_VERBOSE << "\n\n   Welcome to ORCA : an Optimisation-based Framework for Robotics Applications\n";
            setLogLevel(LogLevel::none);
        }

        void Logger::setLogLevel(LogLevel log_level)
        {
            plog::get()->setMaxSeverity(static_cast<plog::Severity>(log_level));
        }

        static Logger __logger__;
    }
}

