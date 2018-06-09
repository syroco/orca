//|  This file is a part of the ORCA framework.
//|
//|  Copyright 2018, Fuzzy Logic Robotics
//|  Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
//|
//|  Main contributor(s): Antoine Hoarau, Ryan Lober, and
//|  Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//|
//|  ORCA is a whole-body reactive controller framework for robotics.
//|
//|  This software is governed by the CeCILL-C license under French law and
//|  abiding by the rules of distribution of free software.  You can  use,
//|  modify and/ or redistribute the software under the terms of the CeCILL-C
//|  license as circulated by CEA, CNRS and INRIA at the following URL
//|  "http://www.cecill.info".
//|
//|  As a counterpart to the access to the source code and  rights to copy,
//|  modify and redistribute granted by the license, users are provided only
//|  with a limited warranty  and the software's author,  the holder of the
//|  economic rights,  and the successive licensors  have only  limited
//|  liability.
//|
//|  In this respect, the user's attention is drawn to the risks associated
//|  with loading,  using,  modifying and/or developing or reproducing the
//|  software by the user in light of its specific status of free software,
//|  that may mean  that it is complicated to manipulate,  and  that  also
//|  therefore means  that it is reserved for developers  and  experienced
//|  professionals having in-depth computer knowledge. Users are therefore
//|  encouraged to load and test the software's suitability as regards their
//|  requirements in conditions enabling the security of their systems and/or
//|  data to be ensured and,  more generally, to use and operate it in the
//|  same conditions as regards security.
//|
//|  The fact that you are presently reading this means that you have had
//|  knowledge of the CeCILL-C license and that you accept its terms.

#pragma once

#include <memory>

namespace orca
{
    namespace common
    {
        struct MutexInterface
        {
            virtual ~MutexInterface() {}
            virtual void lock() = 0;
            virtual void unlock() = 0;
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
                virtual ~MutexRecursive();
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
