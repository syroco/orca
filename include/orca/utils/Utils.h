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
#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <atomic>
#include <chrono>
#include <list>
#include <functional>
#include <thread>
#include <map>
#include "orca/utils/Logger.h"

namespace orca
{
namespace utils
{

// From https://stackoverflow.com/a/348862
class orca_exception : public std::runtime_error {
    std::string msg;
public:
    orca_exception(const std::string &arg, const char *file, int line) :
    std::runtime_error(arg) {
        std::ostringstream o;
        o << "\033[31m" << file << ":" << line << ": " << arg << "\033[0m";
        msg = o.str();
    }
    ~orca_exception() throw() {}
    const char *what() const throw() {
        return msg.c_str();
    }
};
#define orca_throw(arg) throw orca_exception(arg, __FILE__, __LINE__);

class PosixTimer
{
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::nanoseconds nanoseconds;

public:
    const double nsToMs = 1./1e6;
    const double nsToS = 1./1e9;

    explicit PosixTimer(bool start_now = false)
    {
        if (start_now)
            start();
    }
    void start()
    {
        _start = high_resolution_clock::now();
    }
    double elapsedNs() const
    {
        return static_cast<double>(std::chrono::duration_cast<nanoseconds>(high_resolution_clock::now() - _start).count());
    }
    double elapsedMs() const
    {
        return nsToMs * elapsedNs();
    }
    double elapsed() const
    {
        return nsToS * elapsedNs();
    }
    template <typename T, typename Traits>
    friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const PosixTimer& timer)
    {
        return out << timer.elapsed();
    }
private:
    high_resolution_clock::time_point _start;
};



template<class T>
struct UniquePtr
{
    typedef std::unique_ptr<T> Ptr;
    static std::unique_ptr<T> createPtr(){ return std::unique_ptr<T>(new T); }
};

template<class T>
struct SharedPtr
{
    typedef std::shared_ptr<T> Ptr;
    static std::shared_ptr<T> createPtr(){ return std::shared_ptr<T>(new T); }
};

template <typename T>
struct Counter
{
    Counter()
    {
        NrOfInstances++;
    }

    virtual ~Counter()
    {
        --NrOfInstances;
    }
    static std::atomic<int> NrOfInstances;
};
template <typename T> std::atomic<int> Counter<T>::NrOfInstances( 0 );

template <typename T>
class Singleton
{
public:
   static Singleton<T>& getInstance() {
       static Singleton<T> theInstance;
       return theInstance;
   }

private:
   Singleton() {}
   Singleton(const Singleton<T>&);
   Singleton<T>& operator=(const Singleton<T>&);
};

class Formatter
{
public:
    Formatter() {}
    ~Formatter() {}
    template <typename Type>
    Formatter & operator << (const Type & value)
    {
        stream_ << value;
        return *this;
    }

    std::string str() const         { return stream_.str(); }
    operator std::string () const   { return stream_.str(); }

    enum ConvertToString
    {
        to_str
    };
    std::string operator >> (ConvertToString) { return stream_.str(); }

private:
    std::stringstream stream_;

    Formatter(const Formatter &);
    Formatter & operator = (Formatter &);
};
// Usage :
// throw std::runtime_error(Formatter() << foo << 13 << ", bar" << myData);   // implicitly cast to std::string
// throw std::runtime_error(Formatter() << foo << 13 << ", bar" << myData >> Formatter::to_str);    // explicitly cast to std::string



class PeriodicPosixThread
{
public:
    PeriodicPosixThread(std::function<void(void)> f , const unsigned long period_ms, bool start_now = false)
    : f_(f)
    , running_(false)
    , period_ms_(period_ms)
    {
        if(start_now)
            start();
    }

    void start()
    {
        if(!running_)
        {
            running_ = true;
            th_ = std::thread( std::bind(&PeriodicPosixThread::run,this) );
        }
    }

    void run()
    {
        const auto timeWindow = std::chrono::milliseconds(period_ms_);

        while(running_)
        {
            auto start = std::chrono::steady_clock::now();
            f_();
            auto end = std::chrono::steady_clock::now();
            auto elapsed = end - start;

            auto timeToWait = timeWindow - elapsed;
            if(timeToWait > std::chrono::milliseconds::zero())
            {
                std::this_thread::sleep_for(timeToWait);
            }
        }
    }

    void stop()
    {
        if(running_)
        {
            running_ = false;
            th_.join();
        }
    }

    ~PeriodicPosixThread()
    {
        stop();
    }

private:
    std::function<void(void)> f_;
    std::thread th_;
    std::atomic<bool> running_;
    const unsigned long period_ms_;
};

template <typename Derived>
void assertSize(const Eigen::EigenBase<Derived>& a, const Eigen::EigenBase<Derived>& b)
{
    if(a.cols() == b.cols() && a.rows() == b.rows())
        return;
    throw std::length_error(Formatter() << "Size mismatched, provided size (" << a.rows() << " , " << a.cols() << "), but have size (" << b.rows() << " , " << b.cols() << ")");
}

template<class T> bool exists(const std::shared_ptr<T> t, std::list< std::shared_ptr<T> > l){
    return std::find(l.begin(),l.end(),t) != l.end();
}

} // namespace utils
} // namespace orca


// Header Copyright

// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
//
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
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
