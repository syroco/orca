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

        void Logger::parseArgv(int argc, char const* argv[])
        {
            std::vector<std::string> vargv;
            for (size_t i = 0; i < argc; i++)
            {
                vargv.push_back(argv[i]);
            }

            for (size_t i = 0; i < argc; i++)
            {
                if(vargv[i] == "-l" || vargv[i] == "--log_level")
                {
                    if(i + 1 < argc)
                    {
                        Logger::setLogLevel(argv[i+1]);
                    }
                }
            }
        }
        
        void Logger::setLogLevel(const std::string& log_level)
        {
            if(log_level == "verbose")
                Logger::setLogLevel( LogLevel::verbose  );
            if(log_level == "debug")
                Logger::setLogLevel( LogLevel::debug  );
            if(log_level == "info")
                Logger::setLogLevel( LogLevel::info  );
            if(log_level == "warning")
                Logger::setLogLevel( LogLevel::warning  );
            if(log_level == "error")
                Logger::setLogLevel( LogLevel::error  );
            if(log_level == "fatal")
                Logger::setLogLevel( LogLevel::fatal  );
            if(log_level == "none")
                Logger::setLogLevel( LogLevel::none  );
        }
        
        void Logger::setLogLevel(int log_level)
        {
            Logger::setLogLevel(static_cast<LogLevel>(log_level));
        }
        
        void Logger::setLogLevel(LogLevel log_level)
        {
            plog::get()->setMaxSeverity(static_cast<plog::Severity>(log_level));
        }

        static Logger __logger__;
    }
}

#ifdef _WIN32
namespace plog
{
    Record& operator<<(Record& record, const orca::optim::ControlVariable& t)
    {
      record << static_cast<int>(t);
      return record;
    }
}
#endif
