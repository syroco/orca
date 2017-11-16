#pragma once

#if defined(USE_OROCOS_MUTEXES)

#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>
using namespace RTT::os;

#else

#include <mutex>

struct MutexInterface
{
    virtual ~MutexInterface() {}
    virtual void lock() =0;
    virtual void unlock() =0;
    virtual bool trylock() = 0;
};

class Mutex : public MutexInterface
{
public:
    void lock()
    {
        m_.lock();
    }
    void unlock()
    {
        m_.unlock();
    }
    bool trylock()
    {
        return m_.try_lock();
    }
protected:
    std::mutex m_;
};

class MutexRecursive : public MutexInterface
{
public:
    void lock()
    {
        m_.lock();
    }
    void unlock()
    {
        m_.unlock();
    }
    bool trylock()
    {
        return m_.try_lock();
    }
protected:
    std::recursive_mutex m_;
};

class MutexLock
{
public:
    MutexLock( MutexInterface &mutex )
    {
        m_ = &mutex;
        m_->lock();
    }

    ~MutexLock()
    {
        m_->unlock();
    }

protected:
    MutexInterface *m_;
};


#endif