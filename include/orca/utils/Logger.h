#pragma once

#include <plog/Log.h>


namespace orca
{
    namespace utils
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
			static void setLogLevel(int log_level);
			static void setLogLevel(const std::string& log_level);
			static void parseArgv(int argc,char ** argv);
			static void parseArgv(const std::vector<std::string>& v);
			static void parseArgv(int argc,char const* argv[]);
        };
    }
}

#ifdef _WIN32
#include <Eigen/Dense>
#include <sstream>
namespace plog
{
	template <typename Derived>
	Record& operator<<(Record& record, const Eigen::MatrixBase<Derived>& a)
	{
		std::stringstream ss;
		ss << a;
		return record << ss.str();
	}
	template <typename Derived>
	Record& operator<<(Record& record, const Eigen::Transpose<const Derived>& a)
	{
		std::stringstream ss;
		ss << a;
		return record << ss.str();
	}
}
#endif

