#include <orca/common/Mutex.h>
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
                rt_mutex_release(&m_)
            }
            void trylock()
            {
                rt_mutex_acquire(&m,TM_NONBLOCK);
            }
            
            RT_MUTEX m_;
        };
        
        struct MutexRecursive::MutexRecursiveImpl : public Mutex::MutexImpl
        {};
#else
    struct Mutex::MutexImpl
    {
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
            pimpl->m_.lock();
        }
        void Mutex::unlock()
        {
            pimpl->m_.unlock();
        }
        bool Mutex::trylock()
        {
            return pimpl->m_.try_lock();
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
            pimpl->m_.lock();
        }
        void MutexRecursive::unlock()
        {
            pimpl->m_.unlock();
        }
        bool MutexRecursive::trylock()
        {
            return pimpl->m_.try_lock();
        }
    } // namespace common
} // namespace orca
