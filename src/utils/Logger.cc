#include "orca/utils/Logger.h"
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>

namespace plog
{
	class SimpleFormatter
	{
	public:
		static util::nstring header() // This method returns a header for a new file. In our case it is empty.
		{
			return util::nstring();
		}

		static util::nstring format(const Record& record) // This method returns a string from a record.
		{
			util::nostringstream ss;
			ss << record.getMessage() << "\n"; // Produce a simple string with a log message.

			return ss.str();
		}
	};
}

namespace orca
{
    namespace utils
    {
        static plog::RollingFileAppender<plog::SimpleFormatter> fileAppender("orca-log.txt", 8000, 3);
        static plog::ColorConsoleAppender<plog::SimpleFormatter> consoleAppender;

        Logger::Logger()
        {
            plog::init(plog::info, &fileAppender).addAppender(&consoleAppender);
            //LOG_VERBOSE << "\n\n   Welcome to ORCA : an Optimisation-based Framework for Robotics Applications\n";
            setLogLevel(LogLevel::none);
        }
		void Logger::parseArgv(int argc,char ** argv)
        {
			std::vector<std::string> vargv;
			for (int i = 0; i < argc; i++)
				vargv.push_back(argv[i]);
            parseArgv(vargv);
        }
		void Logger::parseArgv(int argc, char const* argv[])
        {
            std::vector<std::string> vargv;
            for (int i = 0; i < argc; i++)
                vargv.push_back(argv[i]);
			parseArgv(vargv);
        }
		void Logger::parseArgv(const std::vector<std::string>& v)
		{
			for (size_t i = 0; i < v.size(); i++)
			{
				if (v[i] == "-l" || v[i] == "--log_level")
				{
					if (i + 1 < v.size())
					{
						Logger::setLogLevel(v[i + 1]);
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
