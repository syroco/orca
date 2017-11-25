#include <orca/common/Mutex.h>
#include <iostream>
#ifdef CONFIG_XENO_VERSION_MAJOR

    #if CONFIG_XENO_VERSION_MAJOR == 3
        // HACK: prevent confliting macros
        #undef debug
        #undef barrier
    #endif

    #include <xeno_config.h> // version number
    
    #if CONFIG_XENO_VERSION_MAJOR == 2
        #include <native/mutex.h>
    #endif
    
    #if CONFIG_XENO_VERSION_MAJOR == 3
        #include <alchemy/mutex.h>
    #endif

#else
    #include <mutex>
#endif

namespace orca
{
    namespace common
    {

#ifdef CONFIG_XENO_VERSION_MAJOR
        struct Mutex::MutexImpl
        {
            MutexImpl()
            {
		rt_mutex_create(&m_,0);
            }
            void lock()
            {
                rt_mutex_acquire(&m_,TM_INFINITE);
            }
            void unlock()
            {
                rt_mutex_release(&m_);
            }
            bool trylock()
            {
                return rt_mutex_acquire(&m_,TM_NONBLOCK);
            }
            
            RT_MUTEX m_;
        };
        
        struct MutexRecursive::MutexRecursiveImpl
	{
	    MutexRecursiveImpl()
            {
                rt_mutex_create(&m_,0);
            }
            void lock()
            {
                rt_mutex_acquire(&m_,TM_INFINITE);
            }
            void unlock()
            {
                rt_mutex_release(&m_);
            }
            bool trylock()
            {
                return rt_mutex_acquire(&m_,TM_NONBLOCK);
            }

            RT_MUTEX m_;
	};
#else
    struct Mutex::MutexImpl
    {
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
        std::mutex m_;
    };

    struct MutexRecursive::MutexRecursiveImpl
    {
        std::recursive_mutex m_;
    };
#endif
        Mutex::Mutex()
        : pimpl(std::make_shared<MutexImpl>())
        {}
            
        void Mutex::lock()
        {
            pimpl->lock();
        }
        void Mutex::unlock()
        {
            pimpl->unlock();
        }
        bool Mutex::trylock()
        {
            return pimpl->trylock();
        }

        Mutex::~Mutex()
        {
            if (trylock())
            {
	            unlock();
	    }
        }

        MutexRecursive::MutexRecursive()
        : pimpl(std::make_shared<MutexRecursiveImpl>())
        {}

        void MutexRecursive::lock()
        {
            pimpl->lock();
        }
        void MutexRecursive::unlock()
        {
            pimpl->unlock();
        }
        bool MutexRecursive::trylock()
        {
            return pimpl->trylock();
        }
	MutexRecursive::~MutexRecursive()
	{
	    if (trylock())
            {
                    unlock();
            }

	}
    } // namespace common
} // namespace orca
