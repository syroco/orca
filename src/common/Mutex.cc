#include <orca/common/Mutex.h>

#include <mutex>

namespace orca
{
    namespace common
    {
        struct Mutex::MutexImpl
        {
            std::mutex m_;
        };
        
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
        
        struct MutexRecursive::MutexRecursiveImpl
        {
            std::recursive_mutex m_;
        };
        
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
