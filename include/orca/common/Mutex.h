#pragma once

#include <memory>

namespace orca
{
    namespace common
    {
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
                Mutex();
                virtual ~Mutex();
                void lock();
                void unlock();
                bool trylock();
            private:
                struct MutexImpl;
                std::shared_ptr<MutexImpl> pimpl;
        };
        
        class MutexRecursive : public MutexInterface
        {
            public:
                MutexRecursive();
                void lock();
                void unlock();
                bool trylock();
            private:
                struct MutexRecursiveImpl;
                std::shared_ptr<MutexRecursiveImpl> pimpl;
        };
        
        class MutexLock
        {
            public:
                /**
                 * Try to lock a Mutex object
                 *
                 * @param mutex The Mutex which should be attempted to be locked
                 */
                MutexLock( MutexInterface &mutex_ )
                : m_(mutex_)
                {
                    m_.lock();
                }
                /**
                 * Releases the lock on the previously locked Mutex
                 */
                ~MutexLock()
                {
                    m_.unlock();
                }

            protected:
                /**
                 * The Mutex to lock and unlock
                 */
                MutexInterface& m_;
        };

        class MutexTryLock
        {
            public:
                /**
                 * Try to lock a Mutex object
                 *
                 * @param mutex The Mutex which should be attempted to be locked
                 */
                MutexTryLock( MutexInterface &mutex )
                        : _mutex( mutex), successful( mutex.trylock() )
                {
                }

                /**
                 * Return if the locking of the Mutex was succesfull
                 *
                 * @return true when the Mutex is locked
                 */
                bool isSuccessful()
                {
                    return successful;
                }

                /**
                 * Releases, if any, a lock on the previously try-locked Mutex
                 */
                ~MutexTryLock()
                {
                    if ( successful )
                        _mutex.unlock();
                }

            protected:
                /**
                 * The Mutex to lock and unlock
                 */
                MutexInterface& _mutex;

            private:
                /**
                 * Stores the state of success
                 */
                bool successful;

        };
    } // namespace common
} // namespace orca









