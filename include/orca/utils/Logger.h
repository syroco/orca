// This file is a part of the ORCA framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// ORCA is a whole-body reactive controller framework for robotics.
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/

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
