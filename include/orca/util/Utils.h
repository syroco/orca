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
#include <orca/math/Utils.h>
#include <orca/util/Logger.h>

namespace orca
{
namespace util
{

class Timer
{
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::nanoseconds nanoseconds;

public:
    explicit Timer(bool run = false)
    {
        if (run)
            Reset();
    }
    void Reset()
    {
        _start = high_resolution_clock::now();
    }
    nanoseconds Elapsed() const
    {
        return std::chrono::duration_cast<nanoseconds>(high_resolution_clock::now() - _start);
    }
    template <typename T, typename Traits>
    friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const Timer& timer)
    {
        return out << timer.Elapsed().count();
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



class PeriodicThread
{
public:
    PeriodicThread(std::function<void(void)> f , const unsigned long period_ms, bool start_now = false)
    : f_(f)
    , do_run_(false)
    , period_ms_(period_ms)
    {
        if(start_now)
            start();
    }

    void start()
    {
        if(!do_run_)
        {
            do_run_ = true;
            th_ = std::thread( std::bind(&PeriodicThread::run,this) );
        }
    }

    void run()
    {
        const auto timeWindow = std::chrono::milliseconds(period_ms_);

        while(do_run_)
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
        if(do_run_)
        {
            do_run_ = false;
            th_.join();
        }
    }

    ~PeriodicThread()
    {
        stop();
    }

private:
    std::function<void(void)> f_;
    std::thread th_;
    std::atomic<bool> do_run_;
    const unsigned long period_ms_;
};

} // namespace util
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
